// Offline-Cache, fest eingefroren: einmal installiert, laeuft die Seite nur aus dem Cache
// (auch ohne Netz). Es gibt KEIN automatisches Update. Findet Chrome eine neue sw.js
// (anderer CACHE-Name), wird sie nur geladen und wartet; aktiv wird sie erst, wenn am Handy
// »Aktualisieren« getippt wird. Neue Version = CACHE hier UND VERSION in app.js hochzaehlen.
const CACHE = "g1phone-v5";
const FILES = ["./", "index.html", "app.js", "proto.js", "manifest.webmanifest", "icon.svg"];

self.addEventListener("install", (e) => {
  // cache: "reload" umgeht den HTTP-Cache, damit die neue Version wirklich frisch ist.
  e.waitUntil(caches.open(CACHE).then((c) =>
    c.addAll(FILES.map((f) => new Request(f, { cache: "reload" })))));
});

self.addEventListener("message", (e) => {
  if (e.data && e.data.type === "activate") self.skipWaiting();
});

self.addEventListener("activate", (e) => {
  e.waitUntil(caches.keys().then((keys) => Promise.all(
    keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)))).then(() => self.clients.claim()));
});

// Nur Cache. Netz nur fuer Dateien, die nicht im Cache sind (wird nicht gespeichert).
self.addEventListener("fetch", (e) => {
  if (e.request.method !== "GET") return;
  e.respondWith(caches.open(CACHE).then((c) => c.match(e.request, { ignoreSearch: true }))
    .then((r) => r || fetch(e.request)));
});
