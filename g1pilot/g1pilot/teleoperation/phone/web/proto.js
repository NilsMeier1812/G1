// Protokoll Handy <-> PC, Gegenstueck zu phone/protocol.py (gleiche Strings,
// gleiche HMACs -- geprueft in test/test_phone_protocol_js.py).
// Laeuft im Browser (Chrome/Android) und unter Node >= 20 (Web Crypto).

export const SERVICE_UUID = "6b1e0001-6d1f-4c3e-9b1a-5f0e7c3a2b10";
export const RX_UUID = "6b1e0002-6d1f-4c3e-9b1a-5f0e7c3a2b10";
export const NOTIFY_UUID = "6b1e0004-6d1f-4c3e-9b1a-5f0e7c3a2b10";
export const TX_UUIDS = [0x10, 0x11, 0x12, 0x13].map(
  (i) => `6b1e00${i.toString(16).padStart(2, "0")}-6d1f-4c3e-9b1a-5f0e7c3a2b10`);
export const CODE_LEN = 16;
const MAC_HEX = 32;

const enc = new TextEncoder();
const dec = new TextDecoder();
const subtle = globalThis.crypto.subtle;

export function hex(bytes) {
  return Array.from(new Uint8Array(bytes), (b) => b.toString(16).padStart(2, "0")).join("");
}

export function unhex(text) {
  const out = new Uint8Array(text.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(text.substr(i * 2, 2), 16);
  return out;
}

export function nonce() {
  const b = new Uint8Array(8);
  globalThis.crypto.getRandomValues(b);
  return hex(b);
}

export function normalizeCode(code) {
  return String(code).toUpperCase().replace(/[^0-9A-Z]/g, "")
    .replace(/O/g, "0").replace(/[IL]/g, "1");
}

async function hmacRaw(keyBytes, text) {
  const key = await subtle.importKey("raw", keyBytes, { name: "HMAC", hash: "SHA-256" },
    false, ["sign"]);
  return new Uint8Array(await subtle.sign("HMAC", key, enc.encode(text)));
}

export async function mac(keyBytes, text) {
  return hex(await hmacRaw(keyBytes, text)).slice(0, MAC_HEX);
}

export const kdf = hmacRaw;

export const codeKey = (code) => enc.encode(normalizeCode(code));
export const pairMac = (code, dev, name, np) => mac(codeKey(code), `pair|${dev}|${name}|${np}`);
export const pairedMac = (code, dev, np) => mac(codeKey(code), `paired|${dev}|${np}`);
export const deviceKey = (code, dev, np) => kdf(codeKey(code), `dev|${dev}|${np}`);
export const serverProof = (dkey, dev, nc, ns) => mac(dkey, `srv|${dev}|${nc}|${ns}`);
export const clientProof = (dkey, dev, nc, ns) => mac(dkey, `cli|${dev}|${nc}|${ns}`);
export const sessionKey = (dkey, nc, ns) => kdf(dkey, `ses|${nc}|${ns}`);
export const sign = (skey, dir, c, payload) => mac(skey, `${dir}|${c}|${payload}`);

export async function encodeSigned(skey, dir, counter, obj) {
  const payload = JSON.stringify(obj);
  return `A${counter}.${await sign(skey, dir, counter, payload)}.${payload}`;
}

function timingSafeEqual(a, b) {
  if (a.length !== b.length) return false;
  let d = 0;
  for (let i = 0; i < a.length; i++) d |= a.charCodeAt(i) ^ b.charCodeAt(i);
  return d === 0;
}

// -> {c, obj} oder null
export async function checkSigned(skey, dir, line) {
  if (typeof line !== "string" || !line.startsWith("A")) return null;
  const i1 = line.indexOf(".");
  const i2 = line.indexOf(".", i1 + 1);
  if (i1 < 0 || i2 < 0) return null;
  const c = parseInt(line.slice(1, i1), 10);
  const m = line.slice(i1 + 1, i2);
  const payload = line.slice(i2 + 1);
  if (!Number.isFinite(c)) return null;
  if (!timingSafeEqual(m, await sign(skey, dir, c, payload))) return null;
  try {
    return { c, obj: JSON.parse(payload) };
  } catch (e) {
    return null;
  }
}

// Seitenkopf "<n>:<i>:<k>:" -> {n, i, k, body(Uint8Array)}
export function parsePage(bytes) {
  const b = new Uint8Array(bytes);
  let colons = 0, pos = 0;
  while (pos < b.length && colons < 3) { if (b[pos] === 58) colons++; pos++; }
  const head = dec.decode(b.slice(0, pos)).split(":");
  return { n: parseInt(head[0], 10), i: parseInt(head[1], 10), k: parseInt(head[2], 10),
    body: b.slice(pos) };
}

// Liste geparster Seiten -> {n, text} oder null (gemischte Staende)
export function joinPages(pages) {
  if (!pages.length) return null;
  const { n, k } = pages[0];
  if (pages.length !== k) return null;
  let len = 0;
  for (let i = 0; i < pages.length; i++) {
    if (pages[i].n !== n || pages[i].i !== i || pages[i].k !== k) return null;
    len += pages[i].body.length;
  }
  const out = new Uint8Array(len);
  let o = 0;
  for (const p of pages) { out.set(p.body, o); o += p.body.length; }
  return { n, text: dec.decode(out) };
}

export const line = (text) => enc.encode(text + "\n");
