// G1 Handy: Bedienung der Demo-GUI ueber Bluetooth LE (Web Bluetooth, Chrome/Android).
// Sicherheit: siehe docs/44_handy_bedienung.md. Kurz:
//  * Geraete-Schluessel aus dem Kopplungscode (nie gesendet), jede Nachricht
//    signiert + Zaehler; der PC prueft Rolle, Steuerhoheit und Statusbezug.
//  * Herzschlag 5 Hz nur bei sichtbarer Seite; Seite weg -> Bedienung abgeben.
//  * Anzeige aelter als 1 s -> alles gesperrt ausser NOT-HALT.
import * as P from "./proto.js";

const $ = (id) => document.getElementById(id);
const STORE = "g1phone.v1";
const HB_MS = 200;
const FRESH_MS = 1000;
const SESSION_LOST_MS = 3000;

let cfg = loadCfg();          // {dev, key(hex), name}
let device = null, chars = null;
let sess = null;              // {skey, c, cin}
let body = null, lastQ = -1, lastAt = 0, lastSigAt = 0;
let preWaiters = [];
let seen = new Set();
let opQueue = Promise.resolve(), opDepth = 0;
let reading = false, readAgain = false;
let wakeLock = null;
let loggingIn = false;

function loadCfg() {
  try { return JSON.parse(localStorage.getItem(STORE)) || null; } catch (e) { return null; }
}
function saveCfg(c) {
  cfg = c;
  try { c ? localStorage.setItem(STORE, JSON.stringify(c)) : localStorage.removeItem(STORE); } catch (e) { /* privat */ }
}

// ── GATT-Zugriffe nacheinander (Web Bluetooth kann nur einen gleichzeitig) ──
function enqueue(fn) {
  opDepth++;
  const p = opQueue.then(fn);
  opQueue = p.catch(() => {}).finally(() => { opDepth--; });
  return p;
}
const writeRaw = (text) => enqueue(() => chars.rx.writeValueWithResponse(P.line(text)));

function sendSigned(obj) {
  if (!sess) return Promise.reject(new Error("keine Sitzung"));
  const s = sess;
  return enqueue(async () => {
    s.c += 1;
    const l = await P.encodeSigned(s.skey, "c2s", s.c, obj);
    await chars.rx.writeValueWithResponse(P.line(l));
  });
}

async function readTx() {
  return enqueue(async () => {
    const page = async (i) => {
      const dv = await chars.tx[i].readValue();
      return P.parsePage(new Uint8Array(dv.buffer, dv.byteOffset, dv.byteLength));
    };
    for (let attempt = 0; attempt < 3; attempt++) {
      const p0 = await page(0);
      const pages = [p0];
      for (let i = 1; i < p0.k; i++) pages.push(await page(i));
      const j = P.joinPages(pages);
      if (j) return j;
    }
    return null;
  });
}

function scheduleRead() {
  if (!chars) return;
  if (reading) { readAgain = true; return; }
  reading = true;
  readTx().then(onTx).catch(() => {}).finally(() => {
    reading = false;
    if (readAgain) { readAgain = false; scheduleRead(); }
  });
}

async function onTx(j) {
  if (!j) return;
  let doc;
  try { doc = JSON.parse(j.text); } catch (e) { return; }
  if (doc.pre && cfgDev() && doc.pre.to === cfgDev()) {
    preWaiters.slice().forEach((w) => w(doc.pre));
  }
  if (doc.a && sess) {
    const got = await P.checkSigned(sess.skey, "s2c", doc.a);
    if (got && got.c > sess.cin) {
      sess.cin = got.c;
      lastSigAt = Date.now();
      onBody(got.obj);
    }
  }
}

let pairingDev = null;
const cfgDev = () => (cfg && cfg.dev) || pairingDev;

// Antwort vor der Anmeldung: nur die zu UNSERER Anfrage (re = unsere Nonce).
function waitPre(types, ms, re) {
  return new Promise((resolve, reject) => {
    const poll = setInterval(scheduleRead, 150);
    const done = (fn, v) => { clearInterval(poll); clearTimeout(t); preWaiters = preWaiters.filter((x) => x !== w); fn(v); };
    const w = (pre) => { if (pre.re === re && types.includes(pre.t)) done(resolve, pre); };
    const t = setTimeout(() => done(reject, new Error("Keine Antwort vom PC.")), ms);
    preWaiters.push(w);
  });
}

// ── Verbinden / Koppeln / Anmelden ───────────────────────────────────────
async function connect() {
  if (!navigator.bluetooth) throw new Error("Kein Web Bluetooth.");
  device = await navigator.bluetooth.requestDevice({ filters: [{ services: [P.SERVICE_UUID] }] });
  device.addEventListener("gattserverdisconnected", onDisconnected);
  setPill("Verbinde …", "busy");
  const server = await device.gatt.connect();
  const svc = await server.getPrimaryService(P.SERVICE_UUID);
  const rx = await svc.getCharacteristic(P.RX_UUID);
  const notify = await svc.getCharacteristic(P.NOTIFY_UUID);
  const tx = [];
  for (const u of P.TX_UUIDS) tx.push(await svc.getCharacteristic(u));
  chars = { rx, notify, tx };
  await notify.startNotifications();
  notify.addEventListener("characteristicvaluechanged", scheduleRead);
  scheduleRead();
}

async function pair(name, code) {
  if (P.normalizeCode(code).length !== P.CODE_LEN) throw new Error("Der Code hat 16 Zeichen.");
  if (!chars) await connect();
  const dev = P.nonce() + P.nonce();
  const np = P.nonce();
  pairingDev = dev;
  const wait = waitPre(["paired", "pair_fail"], 5000, np);
  await writeRaw(JSON.stringify({ t: "pair", dev, name, np, mac: await P.pairMac(code, dev, name, np) }));
  const pre = await wait;
  if (pre.t !== "paired") throw new Error(pre.m || "Kopplung abgelehnt.");
  if (pre.mac !== await P.pairedMac(code, dev, np)) throw new Error("Antwort nicht vom PC mit diesem Code.");
  const key = await P.deviceKey(code, dev, np);
  saveCfg({ dev, key: P.hex(key), name, role: pre.role });
  pairingDev = null;
}

async function login() {
  if (loggingIn || !chars || !cfg) return;
  loggingIn = true;
  try {
    sess = null;
    const dk = P.unhex(cfg.key);
    const nc = P.nonce();
    const wait = waitPre(["chal", "unknown", "busy", "auth_fail"], 4000, nc);
    await writeRaw(JSON.stringify({ t: "hello", dev: cfg.dev, nc }));
    const pre = await wait;
    if (pre.t === "unknown") { saveCfg(null); throw new Error(pre.m || "Nicht mehr gekoppelt."); }
    if (pre.t !== "chal") throw new Error(pre.m || "Anmeldung abgelehnt.");
    if (pre.srv !== await P.serverProof(dk, cfg.dev, nc, pre.ns)) throw new Error("Gegenstelle ist nicht dein PC.");
    const skey = await P.sessionKey(dk, nc, pre.ns);
    sess = { skey, c: 0, cin: 0 };
    lastSigAt = Date.now();
    await writeRaw(JSON.stringify({ t: "auth", dev: cfg.dev, cli: await P.clientProof(dk, cfg.dev, nc, pre.ns) }));
    scheduleRead();
    requestWakeLock();
  } finally {
    loggingIn = false;
    render();
  }
}

function onDisconnected() {
  chars = null; sess = null; body = null; lastQ = -1;
  releaseWakeLock();
  setPill("Getrennt", "");
  render();
}

async function disconnect() {
  try { if (sess) await sendSigned({ t: "bye" }); } catch (e) { /* egal */ }
  if (device && device.gatt.connected) device.gatt.disconnect();
  onDisconnected();
}

// ── Status / Ergebnisse ──────────────────────────────────────────────────
function onBody(b) {
  if (typeof b.q === "number" && b.q > lastQ) { lastQ = b.q; lastAt = Date.now(); }
  body = b;
  for (const r of b.r || []) {
    if (r.i && !seen.has(r.i) && mine.has(r.i)) { seen.add(r.i); toast(r.m, !r.ok); }
  }
  render();
}

const mine = new Set();
function cmd(a, x) {
  if (!sess || (!body && a !== "estop")) return;
  const i = P.nonce();
  mine.add(i);
  sendSigned({ t: "cmd", a, x: x ?? null, s: lastQ, i }).catch((e) => toast("Nicht gesendet: " + e.message, true));
}

// ── Herzschlag + Wachhund ────────────────────────────────────────────────
setInterval(() => {
  if (!sess || document.visibilityState !== "visible") return;
  if (opDepth < 3) sendSigned({ t: "hb" }).catch(() => {});
  if (Date.now() - lastSigAt > SESSION_LOST_MS && chars && !loggingIn) {
    login().catch((e) => toast(e.message, true));      // Sitzung weg -> neu anmelden
  }
}, HB_MS);
setInterval(() => { if (chars) scheduleRead(); }, 400);   // falls Notifys ausbleiben
setInterval(render, 250);

document.addEventListener("visibilitychange", () => {
  if (document.visibilityState !== "visible") {
    // Tasche/Sperrbildschirm/App-Wechsel: Bedienung sofort abgeben.
    if (sess && body && body.me && body.me.ctl) sendSigned({ t: "cmd", a: "release", s: lastQ, i: P.nonce() }).catch(() => {});
    releaseWakeLock();
  } else if (chars && cfg) {
    requestWakeLock();
    if (Date.now() - lastSigAt > SESSION_LOST_MS) login().catch((e) => toast(e.message, true));
  }
});

async function requestWakeLock() {
  try { if ("wakeLock" in navigator && !wakeLock) wakeLock = await navigator.wakeLock.request("screen"); } catch (e) { /* egal */ }
}
function releaseWakeLock() {
  try { if (wakeLock) wakeLock.release(); } catch (e) { /* egal */ }
  wakeLock = null;
}

// ── Oberflaeche ──────────────────────────────────────────────────────────
function setPill(text, cls) { const p = $("pill"); p.textContent = text; p.className = "pill " + (cls || ""); }
const show = (id, on) => $(id).classList.toggle("hidden", !on);

let toastT = null;
function toast(text, bad) {
  const t = $("toast");
  t.textContent = text; t.className = bad ? "bad" : "";
  clearTimeout(toastT);
  toastT = setTimeout(() => t.classList.add("hidden"), 3500);
}

function fresh() { return body && Date.now() - lastAt < FRESH_MS; }

function render() {
  show("unsupported", !navigator.bluetooth);
  const connected = !!chars;
  show("pairView", !cfg);
  show("connectView", !!cfg && !sess);
  show("mainView", !!sess && !!body);
  if (cfg) $("connectInfo").textContent = `Gekoppelt als »${cfg.name}«` + (cfg.role === "view" ? " (nur Ansicht)" : "") + ".";
  $("btnConnect").disabled = loggingIn;
  $("btnConnect").textContent = connected ? "Anmelden" : "Mit G1 verbinden";
  $("estop").disabled = !sess;

  if (!connected) { setPill("Getrennt", ""); return; }
  if (!sess || !body) { setPill(loggingIn ? "Anmelden …" : "Verbunden", "busy"); return; }
  const ok = fresh();
  setPill(ok ? "Live" : "Daten alt", ok ? "ok" : "stale");
  const st = body.st || {}, me = body.me || {};
  show("staleBar", !ok);
  show("estopBanner", !!st.estop);
  const t = $("stText");
  t.textContent = st.txt || "—";
  t.style.borderLeftColor = st.col || "#555";
  $("stInfo").textContent = [st.sim ? "SIM" : "REAL", st.info].filter(Boolean).join(" · ");
  for (const [id, m] of [["chipWalk", "walk"], ["chipGrasp", "grasp"]]) {
    $(id).classList.toggle("on", st.mode === m);
    $(id).classList.toggle("pend", st.pend === m);
  }
  show("chipPad", !!st.pad);
  $("chipPad").textContent = st.pad ? `🎮 ${st.pad}` : "";
  show("chipBusy", !!st.busy);

  const operator = me.role === "operate";
  const ctl = !!me.ctl;
  $("meInfo").textContent = !operator ? "Dieses Handy darf nur ansehen (und NOT-HALT)."
    : ctl ? "Du bedienst. Seite verlassen oder Bildschirm aus = Bedienung abgegeben."
      : "Nur Ansicht. Zum Bedienen übernehmen (der PC sieht das).";
  show("btnTake", operator);
  $("btnTake").textContent = ctl ? "Bedienung abgeben" : "Bedienung übernehmen";
  $("btnTake").disabled = !ok && !ctl;
  show("btnStopMove", operator && !!st.busy);

  const can = operator && ctl && ok;
  show("controls", operator && ctl);
  show("secAck", !!st.ack);
  $("btnAck").textContent = st.estop ? "NOT-HALT quittieren …" : "Start …";
  show("secLoco", !st.sim);
  const loco = st.loco || {};
  $("btnDamp").disabled = !can || !loco.damp;
  $("btnPrep").disabled = !can || !loco.preparation;
  $("btnWalkFsm").disabled = !can || !loco.walk;
  const started = !!st.started && !st.ack;
  $("btnModeWalk").disabled = !can || !started || st.mode === "walk" || st.pend === "walk";
  $("btnModeGrasp").disabled = !can || !started || st.mode === "grasp";
  $("btnAck").disabled = !can;
  const grasp = st.mode === "grasp" && !st.pend;
  show("secGrasp", grasp);
  for (const id of ["btnHandsOpen", "btnHandsClose", "btnLeftClose", "btnRightClose", "btnSafe", "btnHome"]) $(id).disabled = !can;
  renderSeqs(st.seqs || [], can);
}

let seqKey = "";
function renderSeqs(seqs, can) {
  const box = $("seqs");
  const key = JSON.stringify(seqs);
  if (key !== seqKey) {
    seqKey = key;
    box.innerHTML = "";
    for (const name of seqs) {
      const b = document.createElement("button");
      b.textContent = `▶ ${name} …`;
      b.addEventListener("click", () => confirmAction(`Ablauf »${name}«`,
        "Die Arme bewegen sich. Bricht die Verbindung ab, wird der Ablauf gestoppt.", () => cmd("seq", name)));
      box.appendChild(b);
    }
  }
  for (const b of box.querySelectorAll("button")) b.disabled = !can;
}

// ── Bestaetigung per Schieber ────────────────────────────────────────────
let sheetAction = null;
function confirmAction(title, text, fn) {
  $("sheetTitle").textContent = title;
  $("sheetText").textContent = text;
  sheetAction = fn;
  resetSlider();
  show("sheet", true);
}
function closeSheet() { sheetAction = null; show("sheet", false); }
$("sheetCancel").addEventListener("click", closeSheet);

const slider = $("slider"), knob = slider.querySelector(".knob"), fill = slider.querySelector(".fill");
let dragX = null;
function resetSlider() { knob.style.left = "4px"; fill.style.width = "0"; }
slider.addEventListener("pointerdown", (e) => { dragX = e.clientX; slider.setPointerCapture(e.pointerId); });
slider.addEventListener("pointermove", (e) => {
  if (dragX === null) return;
  const max = slider.clientWidth - 64;
  const x = Math.max(0, Math.min(max, e.clientX - dragX));
  knob.style.left = 4 + x + "px";
  fill.style.width = x + 32 + "px";
});
const endDrag = (e) => {
  if (dragX === null) return;
  const max = slider.clientWidth - 64;
  const x = e.clientX - dragX;
  dragX = null;
  if (x >= max * 0.95 && sheetAction) {
    const fn = sheetAction;
    closeSheet();
    if (fresh() && body.me && body.me.ctl) fn();
    else toast("Anzeige nicht aktuell — nichts ausgeführt.", true);
  } else resetSlider();
};
slider.addEventListener("pointerup", endDrag);
slider.addEventListener("pointercancel", () => { dragX = null; resetSlider(); });

// ── Knoepfe ──────────────────────────────────────────────────────────────
const busy = (btn, fn) => async () => {
  btn.disabled = true;
  try { await fn(); } catch (e) { toast(e.message, true); } finally { btn.disabled = false; render(); }
};

$("btnPair").addEventListener("click", busy($("btnPair"), async () => {
  const name = $("pairName").value.trim() || "Handy";
  await pair(name, $("pairCode").value);
  toast("Gekoppelt.");
  await login();
}));
$("btnConnect").addEventListener("click", busy($("btnConnect"), async () => {
  if (!chars) await connect();
  await login();
}));
$("btnForget").addEventListener("click", () => {
  if (confirm("Kopplung auf diesem Handy löschen? Am PC das Gerät ebenfalls entfernen.")) { saveCfg(null); render(); }
});
$("btnDisconnect").addEventListener("click", () => disconnect());

$("estop").addEventListener("click", () => {
  if (!sess) { toast("Keine Verbindung — Controller oder Fernbedienung benutzen!", true); return; }
  cmd("estop");
  if (navigator.vibrate) navigator.vibrate(200);
});
$("btnTake").addEventListener("click", () => cmd(body && body.me && body.me.ctl ? "release" : "take"));
$("btnStopMove").addEventListener("click", () => cmd("cancel"));
$("btnAck").addEventListener("click", () => {
  const st = (body && body.st) || {};
  confirmAction(st.estop ? "NOT-HALT quittieren" : "Start",
    st.sim ? "Quittiert den NOT-HALT. Danach oben einen Modus wählen."
      : "Roboter im Blick, Umfeld frei? Steht der G1 in Damp, schaltet er dabei auf Preparation und wird steif.",
    () => cmd("ack"));
});
$("btnDamp").addEventListener("click", () => confirmAction("Damp",
  "Der G1 wird komplett schlaff und sackt in den Kran.", () => cmd("loco", "damp")));
$("btnPrep").addEventListener("click", () => confirmAction("Preparation",
  "Der G1 wird steif und richtet sich auf. Kran gesichert?", () => cmd("loco", "preparation")));
$("btnWalkFsm").addEventListener("click", () => confirmAction("Walk",
  "Nur mit gelockertem Kran und Bodenkontakt. Der G1 balanciert danach selbst.", () => cmd("loco", "walk")));
$("btnModeWalk").addEventListener("click", () => confirmAction("GEHEN",
  "Die Arme fahren in die Lauf-Pose. Gehen dann mit dem Controller (L2).", () => cmd("mode", "walk")));
$("btnModeGrasp").addEventListener("click", () => confirmAction("GREIFEN",
  "Die Arme werden aktiv geregelt.", () => cmd("mode", "grasp")));
$("btnHandsOpen").addEventListener("click", () => cmd("hand", "open"));
$("btnHandsClose").addEventListener("click", () => cmd("hand", "close"));
$("btnLeftClose").addEventListener("click", () => cmd("hand", "left_close"));
$("btnRightClose").addEventListener("click", () => cmd("hand", "right_close"));
$("btnSafe").addEventListener("click", () => confirmAction("Sichere Pose",
  "Die Arme fahren geplant in die »Sichere Pose«.", () => cmd("safe")));
$("btnHome").addEventListener("click", () => confirmAction("Grundstellung",
  "Die Arme fahren neben den Körper.", () => cmd("home")));

if ("serviceWorker" in navigator) navigator.serviceWorker.register("sw.js").catch(() => {});
render();
