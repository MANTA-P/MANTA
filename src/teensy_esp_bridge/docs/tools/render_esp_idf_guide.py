#!/usr/bin/env python3
"""Render the ESP-IDF HTML guide to an A4 PDF using Qt."""

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import QMarginsF, QUrl
from PyQt5.QtGui import QPageLayout, QPageSize, QTextDocument
from PyQt5.QtPrintSupport import QPrinter
from PyQt5.QtWidgets import QApplication


def main() -> int:
    docs_dir = Path(__file__).resolve().parent.parent
    source = docs_dir / "ESP_IDF_ESP32S3_FLASH_GUIDE.html"
    output = docs_dir / "ESP_IDF_ESP32S3_FLASH_GUIDE.pdf"

    app = QApplication(sys.argv)
    document = QTextDocument()
    document.setBaseUrl(QUrl.fromLocalFile(str(docs_dir) + "/"))
    document.setHtml(source.read_text(encoding="utf-8"))

    printer = QPrinter(QPrinter.HighResolution)
    printer.setOutputFormat(QPrinter.PdfFormat)
    printer.setOutputFileName(str(output))
    printer.setPageLayout(
        QPageLayout(
            QPageSize(QPageSize.A4),
            QPageLayout.Portrait,
            QMarginsF(15, 15, 15, 15),
            QPageLayout.Millimeter,
        )
    )
    printer.setDocName("ESP-IDF로 ESP32-S3 프로젝트 만들기와 플래시")
    document.print_(printer)
    app.quit()

    if not output.exists() or output.stat().st_size == 0:
        raise RuntimeError("PDF output was not created")

    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
