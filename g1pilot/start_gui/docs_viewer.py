"""Eingebauter Doku-Viewer: zeigt die Markdown-Dateien aus g1pilot/docs
gerendert an (Ueberschriften, Tabellen, Code, Bilder, Links).

- Links auf andere .md-Dateien oeffnen im Viewer (inkl. #Anker),
  http(s) im Browser, andere Dateien (Code, Skripte) mit dem System-Standard.
- Links stehen links als Baum nach dem Doku-Schema (Gruppe -> Anleitung/Technik),
  mit Suche ueber Titel UND Inhalt.
- Zurueck/Vor wie im Browser (Alt+Links/Rechts), Strg+F sucht im Dokument.
Kein Browser, kein pip-Paket: Qt rendert Markdown selbst (QTextDocument).
"""
from __future__ import annotations

from pathlib import Path
from urllib.parse import unquote

from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtGui import (QBrush, QColor, QFont, QImageReader, QKeySequence, QShortcut,
                         QTextBlockFormat, QTextCharFormat, QTextCursor,
                         QTextDocument, QTextFormat, QTextFrameFormat, QTextLength,
                         QTextTable)
from PyQt6.QtWidgets import (QHBoxLayout, QLineEdit, QSplitter, QTextBrowser,
                             QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from . import core
from . import theme as T
from .widgets import button, label

MAX_IMG_W = 900


def style_document(doc: QTextDocument, base_pt: float = 11.0, max_img_w: int = MAX_IMG_W) -> None:
    """Nach setMarkdown: Abstaende, Ueberschriften, Code-Bloecke, Tabellen
    lesbar machen (Qt setzt sonst alles sehr eng und ohne Hintergruende)."""
    doc.setDocumentMargin(28)
    cur = QTextCursor(doc)
    block = doc.begin()
    while block.isValid():
        bf = block.blockFormat()
        level = bf.headingLevel()
        is_code = bf.hasProperty(QTextFormat.Property.BlockCodeFence) or bf.nonBreakableLines()
        nf = QTextBlockFormat(bf)
        cf = None
        if level:
            sizes = {1: 2.0, 2: 1.5, 3: 1.22, 4: 1.08}
            nf.setTopMargin({1: 4, 2: 22, 3: 16}.get(level, 12))
            nf.setBottomMargin(8 if level <= 2 else 4)
            cf = QTextCharFormat()
            cf.setFontPointSize(base_pt * sizes.get(level, 1.0))
            cf.setFontWeight(QFont.Weight.Bold if level <= 2 else QFont.Weight.DemiBold)
            cf.setForeground(QBrush(QColor("#ffffff" if level <= 2 else T.FG)))
        elif is_code:
            nf.setBackground(QBrush(QColor("#1c2129")))
            nf.setLeftMargin(0)
            nf.setTopMargin(0)
            nf.setBottomMargin(0)
            nf.setLineHeight(118, 1)   # ProportionalHeight
            cf = QTextCharFormat()
            cf.setForeground(QBrush(QColor("#c9d4e3")))
            nxt = block.next()
            if not (nxt.isValid() and (nxt.blockFormat().nonBreakableLines()
                                       or nxt.blockFormat().hasProperty(
                                           QTextFormat.Property.BlockCodeFence))):
                nf.setBottomMargin(12)
        else:
            nf.setLineHeight(135, 1)
            if bf.bottomMargin() < 8 and not block.textList():
                nf.setBottomMargin(9)
            if block.textList():
                nf.setBottomMargin(4)
        cur.setPosition(block.position())
        cur.setBlockFormat(nf)
        if cf is not None:
            cur.setPosition(block.position())
            cur.setPosition(block.position() + max(0, block.length() - 1),
                            QTextCursor.MoveMode.KeepAnchor)
            cur.mergeCharFormat(cf)
        # Inline-Code `x`: dezente Farbe statt nur Schriftwechsel
        it = block.begin()
        while not it.atEnd():
            frag = it.fragment()
            if frag.isValid():
                f = frag.charFormat()
                if f.fontFixedPitch() and not is_code:
                    c2 = QTextCharFormat()
                    c2.setForeground(QBrush(QColor("#f0b37e")))
                    c2.setBackground(QBrush(QColor("#232934")))
                    cur.setPosition(frag.position())
                    cur.setPosition(frag.position() + frag.length(),
                                    QTextCursor.MoveMode.KeepAnchor)
                    cur.mergeCharFormat(c2)
                if f.isImageFormat():
                    img = f.toImageFormat()
                    size = _image_size(doc, img.name())
                    if size is not None and size.width() > max_img_w:
                        img.setWidth(max_img_w)
                        img.setHeight(size.height() * max_img_w / size.width())
                        cur.setPosition(frag.position())
                        cur.setPosition(frag.position() + frag.length(),
                                        QTextCursor.MoveMode.KeepAnchor)
                        cur.setCharFormat(img)
            it += 1
        block = block.next()
    # Tabellen: feine Linien, Innenabstand, Kopfzeile abgesetzt
    for frame in _frames(doc.rootFrame()):
        if isinstance(frame, QTextTable):
            tf = frame.format()
            tf.setBorder(1)
            tf.setBorderBrush(QBrush(QColor(T.LINE)))
            tf.setBorderStyle(QTextFrameFormat.BorderStyle.BorderStyle_Solid)
            tf.setCellPadding(7)
            tf.setCellSpacing(0)
            if hasattr(tf, "setBorderCollapse"):
                tf.setBorderCollapse(True)
            tf.setWidth(QTextLength(QTextLength.Type.PercentageLength, 100))
            tf.setBottomMargin(12)
            frame.setFormat(tf)
            for c in range(frame.columns()):
                cell = frame.cellAt(0, c)
                cfm = cell.format()
                cfm.setBackground(QBrush(QColor(T.CARD_HI)))
                cell.setFormat(cfm)


def _image_size(doc: QTextDocument, name: str):
    url = doc.baseUrl().resolved(QUrl(name))
    if not url.isLocalFile():
        return None
    r = QImageReader(url.toLocalFile())
    sz = r.size()
    return sz if sz.isValid() else None


def _frames(frame):
    for child in frame.childFrames():
        yield child
        yield from _frames(child)


class DocsWindow(QWidget):
    """Eigenes Fenster (bleibt offen, waehrend man im Startmenue weitermacht)."""

    def __init__(self, open_external=None, docs_dir: Path = core.DOCS_DIR):
        super().__init__()
        self.setWindowTitle("G1 — Dokumentation")
        self.resize(1180, 820)
        self._docs_dir = docs_dir
        self._open_external = open_external or (lambda p: core.open_path(p))
        self._history: list[tuple[Path, str]] = []
        self._pos = -1
        self._current: Path | None = None
        self._items: dict[Path, QTreeWidgetItem] = {}
        self._text_cache: dict[Path, str] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Kopfleiste: Zurueck/Vor, Titel, Suche im Dokument, extern oeffnen
        bar = QWidget()
        bar.setObjectName("bar")
        bar.setStyleSheet(f"QWidget#bar{{background:{T.SURFACE};border-bottom:1px solid {T.LINE};}}")
        bl = QHBoxLayout(bar)
        bl.setContentsMargins(12, 8, 12, 8)
        bl.setSpacing(8)
        self.back_btn = button("‹", "ghost", self.back)
        self.fwd_btn = button("›", "ghost", self.forward)
        for b in (self.back_btn, self.fwd_btn):
            b.setFixedWidth(40)
        bl.addWidget(self.back_btn)
        bl.addWidget(self.fwd_btn)
        self.title_lbl = label("", "")
        f = self.title_lbl.font()
        f.setWeight(QFont.Weight.DemiBold)
        self.title_lbl.setFont(f)
        bl.addWidget(self.title_lbl, 1)
        self.find_edit = QLineEdit()
        self.find_edit.setPlaceholderText("Im Dokument suchen (Strg+F)")
        self.find_edit.setFixedWidth(240)
        self.find_edit.returnPressed.connect(self.find_next)
        bl.addWidget(self.find_edit)
        self.ext_btn = button("Als Datei öffnen", "ghost",
                              lambda: self._current and self._open_external(self._current))
        self.ext_btn.setToolTip("Mit dem System-Programm öffnen (z. B. Texteditor)")
        bl.addWidget(self.ext_btn)
        root.addWidget(bar)

        split = QSplitter(Qt.Orientation.Horizontal)
        split.setHandleWidth(1)
        split.setStyleSheet(f"QSplitter::handle{{background:{T.LINE};}}")
        root.addWidget(split, 1)

        # Links: Suche + Baum
        side = QWidget()
        side.setStyleSheet(f"background:{T.SURFACE};")
        sl = QVBoxLayout(side)
        sl.setContentsMargins(10, 10, 6, 10)
        sl.setSpacing(8)
        self.search = QLineEdit()
        self.search.setPlaceholderText("Doku durchsuchen …")
        self.search.textChanged.connect(self._filter)
        sl.addWidget(self.search)
        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.setIndentation(14)
        self.tree.itemClicked.connect(self._on_item)
        self.tree.itemActivated.connect(self._on_item)
        sl.addWidget(self.tree, 1)
        split.addWidget(side)

        self.view = QTextBrowser()
        self.view.setOpenLinks(False)
        self.view.anchorClicked.connect(self._on_link)
        self.view.document().setDefaultFont(QFont("Inter", 11))
        split.addWidget(self.view)
        split.setSizes([330, 850])
        split.setStretchFactor(1, 1)

        QShortcut(QKeySequence("Alt+Left"), self, activated=self.back)
        QShortcut(QKeySequence("Alt+Right"), self, activated=self.forward)
        QShortcut(QKeySequence.StandardKey.Find, self,
                  activated=lambda: (self.find_edit.setFocus(), self.find_edit.selectAll()))
        QShortcut(QKeySequence("Ctrl+W"), self, activated=self.close)
        QShortcut(QKeySequence("F5"), self, activated=self.reload)

        self._build_tree()
        self._sync_nav()

    # ── Baum ─────────────────────────────────────────────────────────────
    def _build_tree(self) -> None:
        self.tree.clear()
        self._items.clear()
        head, groups = core.docs_index(self._docs_dir)
        bold = QFont(self.tree.font())
        bold.setWeight(QFont.Weight.DemiBold)
        for e in head:
            it = QTreeWidgetItem([e.label])
            it.setData(0, Qt.ItemDataRole.UserRole, str(e.path))
            self.tree.addTopLevelItem(it)
            self._items[e.path] = it
        for g in groups:
            text = g.entry.label if g.entry else g.number
            top = QTreeWidgetItem([text])
            top.setFont(0, bold)
            if g.entry:
                top.setData(0, Qt.ItemDataRole.UserRole, str(g.entry.path))
                self._items[g.entry.path] = top
            self.tree.addTopLevelItem(top)
            for t in g.topics:
                tag = "Anleitung" if t.kind == "A" else "Technik"
                it = QTreeWidgetItem([t.label])
                it.setToolTip(0, f"{t.path.name} · {tag}")
                if t.kind == "T":
                    it.setForeground(0, QBrush(QColor(T.MUTED)))
                it.setData(0, Qt.ItemDataRole.UserRole, str(t.path))
                top.addChild(it)
                self._items[t.path] = it
        self.tree.expandAll()

    def _filter(self, text: str) -> None:
        q = text.strip().lower()

        def match(it: QTreeWidgetItem) -> bool:
            if not q:
                return True
            if q in it.text(0).lower():
                return True
            p = it.data(0, Qt.ItemDataRole.UserRole)
            return bool(p) and q in self._doc_text(Path(p)).lower()

        for i in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(i)
            any_child = False
            for j in range(top.childCount()):
                ch = top.child(j)
                ok = match(ch)
                ch.setHidden(not ok)
                any_child |= ok
            top.setHidden(not (match(top) or any_child))
            if any_child:
                top.setExpanded(True)

    def _doc_text(self, p: Path) -> str:
        if p not in self._text_cache:
            try:
                self._text_cache[p] = p.read_text(encoding="utf-8")
            except OSError:
                self._text_cache[p] = ""
        return self._text_cache[p]

    def _on_item(self, it: QTreeWidgetItem, _col: int = 0) -> None:
        p = it.data(0, Qt.ItemDataRole.UserRole)
        if p:
            self.open_doc(Path(p))

    # ── Anzeigen ─────────────────────────────────────────────────────────
    def open_doc(self, path: Path, anchor: str = "", *, record: bool = True) -> bool:
        path = Path(path).resolve()
        if not path.is_file():
            return False
        if path != self._current:
            md = self._doc_text(path)
            doc = self.view.document()
            doc.setBaseUrl(QUrl.fromLocalFile(str(path.parent) + "/"))
            self.view.setSearchPaths([str(path.parent)])
            self.view.setMarkdown(md)
            style_document(doc, max_img_w=max(300, min(MAX_IMG_W,
                                                       self.view.viewport().width() - 70)))
            self._current = path
            self.title_lbl.setText(core.doc_title(path))
            self.setWindowTitle(f"G1 — Doku — {path.name}")
            it = self._items.get(path)
            if it is not None:
                self.tree.blockSignals(True)
                self.tree.setCurrentItem(it)
                self.tree.blockSignals(False)
        if anchor:
            self.scroll_to_anchor(anchor)
        else:
            self.view.verticalScrollBar().setValue(0)
        if record:
            del self._history[self._pos + 1:]
            self._history.append((path, anchor))
            self._pos = len(self._history) - 1
        self._sync_nav()
        return True

    def scroll_to_anchor(self, anchor: str) -> bool:
        """GitHub-Anker (#abschnitt) auf die passende Ueberschrift abbilden."""
        want = unquote(anchor).lstrip("#").lower()
        doc = self.view.document()
        block = doc.begin()
        seen: dict[str, int] = {}
        while block.isValid():
            if block.blockFormat().headingLevel():
                slug = core.github_slug(block.text())
                n = seen.get(slug, 0)
                seen[slug] = n + 1
                if n:
                    slug = f"{slug}-{n}"
                if slug == want:
                    self.view.document().documentLayout()  # Layout sicher vorhanden
                    y = self.view.document().documentLayout().blockBoundingRect(block).top()
                    cur = QTextCursor(block)
                    self.view.setTextCursor(cur)
                    self.view.verticalScrollBar().setValue(int(y) - 8)
                    return True
            block = block.next()
        return False

    def _on_link(self, url: QUrl) -> None:
        if url.scheme() in ("http", "https", "mailto"):
            core.open_url(url.toString())
            return
        frag = url.fragment()
        if url.scheme() in ("", "file") and not url.path() and frag:
            # Anker im selben Dokument
            self.scroll_to_anchor(frag)
            if self._current is not None:
                del self._history[self._pos + 1:]
                self._history.append((self._current, frag))
                self._pos = len(self._history) - 1
                self._sync_nav()
            return
        local = url.toLocalFile() if url.isLocalFile() else unquote(url.path())
        p = Path(local)
        if not p.is_absolute() and self._current is not None:
            p = (self._current.parent / p)
        p = p.resolve()
        if p.suffix.lower() == ".md" and p.is_file():
            self.open_doc(p, frag)
        elif p.exists():
            self._open_external(p)

    def reload(self) -> None:
        if self._current is None:
            return
        cur = self._current
        self._text_cache.pop(cur, None)
        pos = self.view.verticalScrollBar().value()
        self._current = None
        self.open_doc(cur, record=False)
        self.view.verticalScrollBar().setValue(pos)

    def back(self) -> None:
        if self._pos > 0:
            self._pos -= 1
            p, a = self._history[self._pos]
            self.open_doc(p, a, record=False)

    def forward(self) -> None:
        if self._pos < len(self._history) - 1:
            self._pos += 1
            p, a = self._history[self._pos]
            self.open_doc(p, a, record=False)

    def _sync_nav(self) -> None:
        self.back_btn.setEnabled(self._pos > 0)
        self.fwd_btn.setEnabled(self._pos < len(self._history) - 1)
        self.ext_btn.setEnabled(self._current is not None)

    def find_next(self) -> None:
        q = self.find_edit.text()
        if not q:
            return
        if not self.view.find(q):
            c = self.view.textCursor()
            c.movePosition(QTextCursor.MoveOperation.Start)
            self.view.setTextCursor(c)
            self.view.find(q)
