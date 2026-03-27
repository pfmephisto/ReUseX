#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Create a print-ready PDF with AprilTag 36h11 tags.

See docs/guides/apriltag-36h11-printing.md for usage guide.

Usage:
    python create_apriltag_pdf.py --input tags/ --output apriltags.pdf --tag-size 100 --cols 2 --rows 3
"""

import argparse
from pathlib import Path
from reportlab.lib.pagesizes import A4, letter
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


def create_apriltag_pdf(tag_dir, output_pdf, tag_size_mm=100, cols=2, rows=3,
                        page_size=A4, margin_mm=20, add_labels=True):
    """
    Create a PDF with AprilTags arranged in a grid.

    Args:
        tag_dir: Directory containing PNG tag images
        output_pdf: Output PDF filename
        tag_size_mm: Size of each tag in millimeters
        cols: Number of columns per page
        rows: Number of rows per page
        page_size: Page size (A4 or letter)
        margin_mm: Page margin in millimeters
        add_labels: Whether to add tag ID labels below each tag
    """
    # Accept both official repo filenames and generated filenames
    tag_files = sorted(Path(tag_dir).glob("tag36_11_*.png"))
    tag_files += sorted(Path(tag_dir).glob("tag36h11_*.png"))
    # Deduplicate (in case both patterns match) and sort
    tag_files = sorted(set(tag_files), key=lambda p: p.name)

    if not tag_files:
        print(f"Error: No tag images found in {tag_dir}")
        print("  Expected filenames: tag36_11_*.png or tag36h11_*.png")
        return

    print(f"Found {len(tag_files)} tag images")

    # Setup PDF
    c = canvas.Canvas(str(output_pdf), pagesize=page_size)
    page_width, page_height = page_size

    # Convert mm to points
    tag_size = tag_size_mm * mm
    margin = margin_mm * mm
    label_height = 5 * mm if add_labels else 0
    cell_height = tag_size + label_height + 5 * mm
    cell_width = tag_size + 10 * mm

    # Calculate grid positioning
    total_width = cols * cell_width
    start_x = (page_width - total_width) / 2
    start_y = page_height - margin - tag_size

    tags_per_page = cols * rows
    tag_count = 0
    page_num = 1

    for tag_file in tag_files:
        # Calculate position in grid
        idx_on_page = tag_count % tags_per_page
        row = idx_on_page // cols
        col = idx_on_page % cols

        # Start new page if needed
        if tag_count > 0 and idx_on_page == 0:
            # Add page number to the page we just finished
            _draw_page_number(c, page_width, page_num)
            c.showPage()
            page_num += 1

        # Calculate tag position
        x = start_x + col * cell_width
        y = start_y - row * cell_height

        # Draw tag
        c.drawImage(str(tag_file), x, y, width=tag_size, height=tag_size,
                    preserveAspectRatio=True, mask='auto')

        # Add label
        if add_labels:
            tag_id = tag_file.stem.split('_')[-1]
            label = f"36h11 ID: {tag_id}"
            c.setFont("Helvetica", 8)
            text_width = c.stringWidth(label, "Helvetica", 8)
            c.drawString(x + (tag_size - text_width) / 2, y - 3 * mm, label)

        # Draw crop marks
        mark_length = 5 * mm
        c.setLineWidth(0.5)
        c.setStrokeColorRGB(0.7, 0.7, 0.7)
        # Top-left
        c.line(x - mark_length, y + tag_size, x, y + tag_size)
        c.line(x, y + tag_size, x, y + tag_size + mark_length)
        # Top-right
        c.line(x + tag_size, y + tag_size + mark_length,
               x + tag_size, y + tag_size)
        c.line(x + tag_size, y + tag_size,
               x + tag_size + mark_length, y + tag_size)

        tag_count += 1

    # Add page number to the last page and save
    if tag_count > 0:
        _draw_page_number(c, page_width, page_num)

    c.save()
    print(f"\nCreated PDF: {output_pdf}")
    print(f"  Total tags: {tag_count}")
    print(f"  Total pages: {page_num}")
    print(f"  Tag size: {tag_size_mm}mm")


def _draw_page_number(c, page_width, page_num):
    """Draw page number at the bottom center of the current page."""
    c.setFont("Helvetica", 10)
    text = f"Page {page_num}"
    text_width = c.stringWidth(text, "Helvetica", 10)
    c.drawString((page_width - text_width) / 2, 10 * mm, text)


def main():
    parser = argparse.ArgumentParser(
        description='Create AprilTag PDF for printing')
    parser.add_argument('--input', type=str, required=True,
                        help='Directory with tag images')
    parser.add_argument('--output', type=str,
                        default='apriltags_36h11.pdf', help='Output PDF file')
    parser.add_argument('--tag-size', type=float,
                        default=100, help='Tag size in mm')
    parser.add_argument('--cols', type=int, default=2, help='Columns per page')
    parser.add_argument('--rows', type=int, default=3, help='Rows per page')
    parser.add_argument('--page', type=str, default='A4',
                        choices=['A4', 'letter'], help='Page size')
    parser.add_argument('--margin', type=float, default=20,
                        help='Page margin in mm')
    parser.add_argument('--no-labels', action='store_true',
                        help='Omit tag ID labels')

    args = parser.parse_args()

    page_size = A4 if args.page == 'A4' else letter

    create_apriltag_pdf(
        tag_dir=args.input,
        output_pdf=args.output,
        tag_size_mm=args.tag_size,
        cols=args.cols,
        rows=args.rows,
        page_size=page_size,
        margin_mm=args.margin,
        add_labels=not args.no_labels
    )


if __name__ == '__main__':
    main()
