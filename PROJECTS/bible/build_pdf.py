#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import textwrap
from datetime import datetime, timezone
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.pdfgen import canvas

SRC = Path("PROJECTS/reports/WOODEN_ARMATURE_BIBLE.md")
OUT = Path("PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf")
REPORT = Path("PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf_report.json")

W, H = letter
ML, MR, MT, MB = 0.82 * inch, 0.72 * inch, 0.86 * inch, 0.72 * inch
CW = W - ML - MR
PARCHMENT = colors.HexColor("#F4EAD2")
INK = colors.HexColor("#241811")
REDWOOD = colors.HexColor("#6E2F1A")
BRASS = colors.HexColor("#B08A3C")
VELLUM = colors.HexColor("#FFF8E7")
BARK = colors.HexColor("#3B2115")


def plain(s: str) -> str:
    reps = {"\u2014": " - ", "\u2013": "-", "\u2018": "'", "\u2019": "'", "\u201c": '"', "\u201d": '"', "\u2192": "->"}
    for a, b in reps.items():
        s = s.replace(a, b)
    s = re.sub(r"`([^`]*)`", r"\1", s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"\1", s)
    s = re.sub(r"\*([^*]+)\*", r"\1", s)
    return s.encode("latin-1", "ignore").decode("latin-1")


class Pdf:
    def __init__(self):
        OUT.parent.mkdir(parents=True, exist_ok=True)
        self.c = canvas.Canvas(str(OUT), pagesize=letter)
        self.page = 0
        self.y = H - MT
        self.h1 = 0

    def bg(self):
        c = self.c
        c.setFillColor(PARCHMENT)
        c.rect(0, 0, W, H, fill=1, stroke=0)
        c.setStrokeColor(REDWOOD)
        c.setLineWidth(1.2)
        c.rect(0.38 * inch, 0.38 * inch, W - 0.76 * inch, H - 0.76 * inch)
        c.setStrokeColor(BRASS)
        c.setLineWidth(0.55)
        c.rect(0.47 * inch, 0.47 * inch, W - 0.94 * inch, H - 0.94 * inch)

    def new_page(self):
        if self.page:
            self.c.showPage()
        self.page += 1
        self.bg()
        self.y = H - MT
        if self.page > 1:
            self.c.setFillColor(BARK)
            self.c.setFont("Helvetica", 7.5)
            self.c.drawString(ML, H - 0.55 * inch, "THE WOODEN ARMATURE BIBLE")
            self.c.drawRightString(W - MR, H - 0.55 * inch, "PHY / REDWOOD / ARCHIVE")
            self.c.setStrokeColor(BRASS)
            self.c.line(ML, H - 0.62 * inch, W - MR, H - 0.62 * inch)
            self.c.drawString(ML, 0.43 * inch, "MIND | BODY | SOUL")
            self.c.drawRightString(W - MR, 0.43 * inch, str(self.page))
            self.y = H - 0.9 * inch

    def ensure(self, amt):
        if self.y - amt < MB:
            self.new_page()

    def title(self):
        self.new_page()
        c = self.c
        c.setFillColor(REDWOOD)
        c.setFont("Times-Bold", 28)
        c.drawCentredString(W / 2, H - 2.1 * inch, "THE WOODEN")
        c.drawCentredString(W / 2, H - 2.55 * inch, "ARMATURE BIBLE")
        c.setStrokeColor(BRASS)
        c.setLineWidth(1.3)
        c.line(1.25 * inch, H - 2.85 * inch, W - 1.25 * inch, H - 2.85 * inch)
        c.setFillColor(BARK)
        c.setFont("Times-Italic", 13)
        c.drawCentredString(W / 2, H - 3.25 * inch, "A PHY-compatible fabrication reference")
        c.setFillColor(REDWOOD)
        c.setFont("Times-Bold", 16)
        c.drawCentredString(W / 2, H - 4.15 * inch, "MIND | BODY | SOUL")
        c.setStrokeColor(BRASS)
        c.circle(W / 2, H - 5.35 * inch, 0.65 * inch)
        c.setFillColor(BARK)
        c.setFont("Times-Bold", 8.5)
        c.drawCentredString(W / 2, H - 5.2 * inch, "BONES FIRST")
        c.drawCentredString(W / 2, H - 5.35 * inch, "TRUTH FIRST")
        c.drawCentredString(W / 2, H - 5.5 * inch, "RECORDS FIRST")
        c.setFont("Times-Italic", 10)
        c.drawCentredString(W / 2, 1.15 * inch, datetime.now(timezone.utc).strftime("Generated UTC %Y-%m-%d %H:%M:%S"))
        self.new_page()

    def para(self, s, font="Times-Roman", size=10.2, lead=13.4, indent=0):
        s = plain(s).strip()
        if not s:
            self.y -= 4
            return
        lines = textwrap.wrap(s, width=max(40, int((CW-indent)/(size*0.48))), break_long_words=False)
        self.ensure(len(lines) * lead + 4)
        self.c.setFont(font, size)
        self.c.setFillColor(INK)
        for line in lines:
            if self.y - lead < MB:
                self.new_page()
                self.c.setFont(font, size)
                self.c.setFillColor(INK)
            self.c.drawString(ML + indent, self.y, line)
            self.y -= lead
        self.y -= 2

    def heading(self, s, level):
        s = plain(s)
        if level == 1:
            self.h1 += 1
            if self.page > 2:
                self.new_page()
            self.c.setStrokeColor(BRASS)
            self.c.line(ML, self.y, W - MR, self.y)
            self.y -= 18
            self.c.setFillColor(REDWOOD)
            self.c.setFont("Times-Bold", 18)
            for line in textwrap.wrap(s.upper(), width=48):
                self.c.drawString(ML, self.y, line)
                self.y -= 22
            self.c.setStrokeColor(BRASS)
            self.c.line(ML, self.y, W - MR, self.y)
            self.y -= 14
        elif level == 2:
            self.ensure(40)
            self.c.setFillColor(REDWOOD)
            self.c.setFont("Times-Bold", 14)
            self.c.drawString(ML, self.y, s[:85])
            self.y -= 22
        else:
            self.ensure(26)
            self.c.setFillColor(BARK)
            self.c.setFont("Times-Bold", 11)
            self.c.drawString(ML, self.y, s[:95])
            self.y -= 16

    def code(self, lines):
        if not lines:
            return
        for i in range(0, len(lines), 24):
            chunk = lines[i:i+24]
            h = len(chunk) * 10 + 12
            self.ensure(h + 4)
            self.c.setFillColor(VELLUM)
            self.c.setStrokeColor(BRASS)
            self.c.roundRect(ML - 4, self.y - h + 6, CW + 8, h, 4, fill=1, stroke=1)
            self.c.setFillColor(BARK)
            self.c.setFont("Courier", 8)
            y = self.y - 8
            for raw in chunk:
                self.c.drawString(ML + 4, y, plain(raw)[:100])
                y -= 10
            self.y = y - 8

    def save(self):
        self.c.save()


def main():
    pdf = Pdf()
    pdf.title()
    in_code = False
    code = []
    for raw in SRC.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if line.startswith("<!--") or line == "-->":
            continue
        if line.strip().startswith("```"):
            if in_code:
                pdf.code(code); code = []; in_code = False
            else:
                in_code = True
            continue
        if in_code:
            code.append(line); continue
        if line.strip() == "---":
            pdf.y -= 8; continue
        m = re.match(r"^(#{1,6})\s+(.*)$", line)
        if m:
            pdf.heading(m.group(2), len(m.group(1)))
        elif line.strip().startswith("- "):
            pdf.para("- " + line.strip()[2:], size=9.8, lead=12.6, indent=0.12*inch)
        elif line.strip().startswith("|"):
            pdf.code([line])
        else:
            pdf.para(line)
    if in_code:
        pdf.code(code)
    pdf.save()
    report = {"input": str(SRC), "output": str(OUT), "pages": pdf.page, "top_level_sections_seen": pdf.h1, "style": "redwood_pdf_v1", "generated_utc": datetime.now(timezone.utc).isoformat()}
    REPORT.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
