# AprilTag 36h11 Generation and Printing Guide

## Overview

AprilTags are fiducial markers designed for robust detection in computer vision applications. The **36h11** family is one of the most commonly used tag families, offering a good balance between robustness (11 Hamming distance) and the number of unique tags (587 tags).

This guide explains how to generate AprilTag 36h11 tags and create print-ready PDFs for use in scanning and SLAM applications with ReUseX.

## Why AprilTags 36h11?

**Advantages:**
- **High robustness**: Hamming distance of 11 allows detection even with occlusion or noise
- **Sufficient variety**: 587 unique tags for most applications
- **Standard format**: Widely supported in robotics and computer vision libraries
- **SLAM integration**: Excellent for pose estimation and loop closure detection

**Use cases in ReUseX:**
- Ground control points for scan registration
- Scale reference markers
- Room identification markers
- Multi-scan alignment anchors

## Prerequisites

### Software Options

#### Option 1: Python with pupil-apriltags
```bash
pip install pupil-apriltags numpy pillow reportlab
```

#### Option 2: apriltag-gen (Command-line tool)
```bash
# Clone the AprilTag generation repository
git clone https://github.com/AprilRobotics/apriltag-imgs.git
cd apriltag-imgs
```

#### Option 3: Online Generator
Use the online generator at: https://apriltag-generation.com/

## Generating AprilTag Images

### Method 1: Python Script (Recommended)

Create a Python script `generate_apriltags.py`:

```python
#!/usr/bin/env python3
"""
Generate AprilTag 36h11 images as PNG files.

Usage:
    python generate_apriltags.py --start 0 --count 20 --size 800 --border 1 --output tags/
"""

import argparse
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pathlib import Path

def generate_apriltag_image(tag_id, size_px=800, border_bits=1, family="36h11"):
    """
    Generate a basic AprilTag image.

    Note: This creates a placeholder. For production use, download pre-rendered
    tags from the official AprilTag repository or use the pupil-apriltags library.
    """
    # For actual tag generation, use the official AprilTag repository images
    # This is a placeholder that creates a border and ID label

    img = Image.new('L', (size_px, size_px), 255)
    draw = ImageDraw.Draw(img)

    # Add border
    border_width = int(size_px * 0.1)
    draw.rectangle([0, 0, size_px-1, size_px-1], outline=0, width=border_width)

    # Add tag ID text (for reference only - not part of actual tag)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 40)
    except:
        font = ImageFont.load_default()

    text = f"Tag {family}\nID: {tag_id}"
    bbox = draw.textbbox((0, 0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    position = ((size_px - text_width) // 2, size_px - text_height - 20)
    draw.text(position, text, fill=0, font=font)

    return img

def main():
    parser = argparse.ArgumentParser(description='Generate AprilTag 36h11 images')
    parser.add_argument('--start', type=int, default=0, help='Starting tag ID')
    parser.add_argument('--count', type=int, default=20, help='Number of tags to generate')
    parser.add_argument('--size', type=int, default=800, help='Image size in pixels')
    parser.add_argument('--border', type=int, default=1, help='Border size in bits')
    parser.add_argument('--output', type=str, default='tags/', help='Output directory')
    parser.add_argument('--family', type=str, default='36h11', help='Tag family')

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Generating {args.count} AprilTags (family: {args.family})")
    print(f"Tag IDs: {args.start} to {args.start + args.count - 1}")
    print(f"Output directory: {output_dir}")

    for i in range(args.count):
        tag_id = args.start + i
        img = generate_apriltag_image(tag_id, args.size, args.border, args.family)

        filename = f"tag{args.family}_{tag_id:05d}.png"
        filepath = output_dir / filename
        img.save(filepath)
        print(f"  Generated: {filename}")

    print(f"\nDone! Generated {args.count} tags in {output_dir}")

if __name__ == '__main__':
    main()
```

**Important Note:** For production use, download official pre-rendered tags from:
```bash
git clone https://github.com/AprilRobotics/apriltag-imgs.git
cd apriltag-imgs/tag36h11
# Use the PNG files directly
```

### Method 2: Download Official Tags

The official AprilTag repository provides pre-rendered, high-quality tag images:

```bash
# Download official tag images
wget https://github.com/AprilRobotics/apriltag-imgs/archive/refs/heads/master.zip
unzip master.zip
cd apriltag-imgs-master/tag36h11/

# Tag files are named: tag36_11_00000.png, tag36_11_00001.png, etc.
ls -1 *.png | head -10
```

## Creating Print-Ready PDFs

### Method 1: Python Script with ReportLab

Create `create_apriltag_pdf.py`:

```python
#!/usr/bin/env python3
"""
Create a print-ready PDF with AprilTag 36h11 tags.

Usage:
    python create_apriltag_pdf.py --input tags/ --output apriltags.pdf --tag-size 100 --cols 2 --rows 3
"""

import argparse
from pathlib import Path
from reportlab.lib.pagesizes import A4, letter
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader
from PIL import Image

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
    # Get all tag files
    tag_files = sorted(Path(tag_dir).glob("tag36_11_*.png"))
    if not tag_files:
        tag_files = sorted(Path(tag_dir).glob("tag36h11_*.png"))

    if not tag_files:
        print(f"Error: No tag images found in {tag_dir}")
        return

    print(f"Found {len(tag_files)} tag images")

    # Setup PDF
    c = canvas.Canvas(str(output_pdf), pagesize=page_size)
    page_width, page_height = page_size

    # Convert mm to points
    tag_size = tag_size_mm * mm
    margin = margin_mm * mm
    label_height = 5 * mm if add_labels else 0
    cell_height = tag_size + label_height + 5*mm
    cell_width = tag_size + 10*mm

    # Calculate grid positioning
    total_width = cols * cell_width
    total_height = rows * cell_height
    start_x = (page_width - total_width) / 2
    start_y = page_height - margin - tag_size

    tag_count = 0
    page_num = 1

    for tag_file in tag_files:
        # Calculate position in grid
        row = (tag_count % (cols * rows)) // cols
        col = (tag_count % (cols * rows)) % cols

        # Start new page if needed
        if tag_count > 0 and tag_count % (cols * rows) == 0:
            c.showPage()
            page_num += 1
            print(f"  Created page {page_num}")

        # Calculate tag position
        x = start_x + col * cell_width
        y = start_y - row * cell_height

        # Draw tag
        c.drawImage(str(tag_file), x, y, width=tag_size, height=tag_size,
                   preserveAspectRatio=True, mask='auto')

        # Add label
        if add_labels:
            # Extract tag ID from filename
            tag_id = tag_file.stem.split('_')[-1]
            label = f"36h11 ID: {tag_id}"
            c.setFont("Helvetica", 8)
            text_width = c.stringWidth(label, "Helvetica", 8)
            c.drawString(x + (tag_size - text_width) / 2, y - 3*mm, label)

        # Draw crop marks (optional)
        mark_length = 5 * mm
        c.setLineWidth(0.5)
        c.setStrokeColorRGB(0.7, 0.7, 0.7)
        # Top-left
        c.line(x - mark_length, y + tag_size, x, y + tag_size)
        c.line(x, y + tag_size, x, y + tag_size + mark_length)
        # Top-right
        c.line(x + tag_size, y + tag_size + mark_length, x + tag_size, y + tag_size)
        c.line(x + tag_size, y + tag_size, x + tag_size + mark_length, y + tag_size)

        tag_count += 1

    # Add page numbers
    for i in range(1, page_num + 1):
        c.setFont("Helvetica", 10)
        c.drawString(page_width / 2, 10*mm, f"Page {i}")
        if i < page_num:
            c.showPage()

    c.save()
    print(f"\nCreated PDF: {output_pdf}")
    print(f"Total tags: {tag_count}")
    print(f"Total pages: {page_num}")
    print(f"Tag size: {tag_size_mm}mm")

def main():
    parser = argparse.ArgumentParser(description='Create AprilTag PDF for printing')
    parser.add_argument('--input', type=str, required=True, help='Directory with tag images')
    parser.add_argument('--output', type=str, default='apriltags_36h11.pdf', help='Output PDF file')
    parser.add_argument('--tag-size', type=float, default=100, help='Tag size in mm')
    parser.add_argument('--cols', type=int, default=2, help='Columns per page')
    parser.add_argument('--rows', type=int, default=3, help='Rows per page')
    parser.add_argument('--page', type=str, default='A4', choices=['A4', 'letter'], help='Page size')
    parser.add_argument('--margin', type=float, default=20, help='Page margin in mm')
    parser.add_argument('--no-labels', action='store_true', help='Omit tag ID labels')

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
```

**Usage example:**
```bash
# Create PDF with 100mm tags, 2x3 grid on A4 paper
python create_apriltag_pdf.py --input apriltag-imgs-master/tag36h11/ \
    --output apriltags_36h11_100mm.pdf --tag-size 100 --cols 2 --rows 3
```

### Method 2: ImageMagick + img2pdf

Quick command-line approach:

```bash
# Resize tags to desired print size (300 DPI)
# For 100mm tags at 300 DPI: 100mm * 300 DPI / 25.4 = 1181 pixels
mkdir -p tags_resized
for f in apriltag-imgs-master/tag36h11/tag36_11_000*.png; do
    convert "$f" -resize 1181x1181 -bordercolor white -border 50 \
            "tags_resized/$(basename $f)"
done

# Create PDF (requires img2pdf)
img2pdf tags_resized/*.png -o apriltags_36h11.pdf
```

## Printing Best Practices

### Print Settings

**Critical settings:**
- **Scale**: 100% (actual size) - no scaling or "fit to page"
- **Quality**: Best/High quality
- **Color**: Black and white, grayscale sufficient
- **Paper**: White, non-glossy matte paper (glossy can cause reflections)
- **DPI**: Minimum 300 DPI for tags 100mm or smaller
- **Printer**: Laser printer recommended (inkjet acceptable if high quality)

### Printing Checklist

1. **Test print first**: Print a single tag to verify sizing
   ```bash
   # Measure the printed tag with a ruler - should match specified size
   ```

2. **Verify scale**: The tag should be exactly the specified size (e.g., 100mm)

3. **Check contrast**: Tags must have sharp black/white contrast
   - No fading or graying
   - No bleeding of ink
   - Clean edges

4. **Mounting**:
   - Mount on rigid backing (foam board, cardboard, acrylic)
   - Ensure tag is completely flat (no warping or curling)
   - Use spray adhesive for best results
   - Avoid reflective lamination (causes detection issues)

### Tag Size Guidelines

| Use Case | Recommended Size | Detection Range |
|----------|-----------------|-----------------|
| Close-range scanning | 50-80mm | 0.5-2m |
| Standard scanning | 100-150mm | 1-5m |
| Large spaces | 200-300mm | 3-10m |
| Ground control points | 150-250mm | 2-8m |

**Rule of thumb**: Tag should be at least 10 pixels across in the image for reliable detection.

## Using AprilTags with ReUseX

### Integration with RTABMap SLAM

AprilTags can be detected during SLAM and used for:

1. **Loop closure**: Improve mapping accuracy
2. **Scale reference**: Provide known distances
3. **Coordinate alignment**: Establish reference frames

### Example Workflow

```bash
# 1. Generate and print tags (IDs 0-19)
python create_apriltag_pdf.py --input tags/ --output my_tags.pdf --tag-size 150

# 2. Place tags in scanning environment
# - Attach to walls at known locations
# - Record tag positions and IDs

# 3. Capture SLAM data with RTABMap
# - Tags will be automatically detected if enabled
# - Store tag detections in database

# 4. Process with ReUseX
rux import rtabmap scan.db
# Tags are available for registration and alignment
```

### Tag Placement Tips

**For SLAM loop closure:**
- Place tags at room entrances/exits
- Ensure tags are visible from multiple viewpoints
- Space tags 3-5 meters apart
- Mount at camera height (1.5m typical)

**For ground control:**
- Place at room corners
- Record precise XYZ coordinates
- Use larger tags (200mm+) for better accuracy
- Photograph tags with measuring tape for verification

## Calibration and Verification

### Verify Tag Detection

Test your printed tags before deployment:

```python
#!/usr/bin/env python3
"""Quick test script to verify tag detection"""
import cv2
from pupil_apriltags import Detector

# Initialize detector
detector = Detector(families='tag36h11')

# Capture image
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Detect tags
results = detector.detect(gray)
print(f"Detected {len(results)} tags:")
for r in results:
    print(f"  Tag ID {r.tag_id} at center {r.center}")

cap.release()
```

### Common Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No detection | Low contrast | Use higher quality printer, adjust brightness |
| Partial detection | Warped/curved | Mount on rigid flat surface |
| Intermittent detection | Reflections | Use matte paper, avoid glossy lamination |
| Wrong ID detected | Poor print quality | Increase DPI, use laser printer |
| Unstable pose | Too small | Use larger tags for your distance |

## Resources

**Official AprilTag:**
- Repository: https://github.com/AprilRobotics/apriltag
- Images: https://github.com/AprilRobotics/apriltag-imgs
- Paper: https://april.eecs.umich.edu/software/apriltag

**Detection Libraries:**
- pupil-apriltags (Python): https://github.com/pupil-labs/apriltags
- apriltag (C++): Official implementation
- OpenCV: `cv::aruco::detectMarkers()` (supports AprilTag)

**Online Generators:**
- https://apriltag-generation.com/
- https://chev.me/arucogen/ (includes AprilTag families)

## Summary

1. **Download** official 36h11 tag images from GitHub
2. **Generate PDF** using Python script or ImageMagick
3. **Print** at 100% scale, minimum 300 DPI, matte paper
4. **Verify** size with ruler (should match specified dimensions)
5. **Mount** on rigid backing, keep flat
6. **Place** strategically in environment
7. **Test** detection before scanning
8. **Document** tag IDs and positions for reference

For ReUseX integration, refer to the RTABMap documentation for enabling AprilTag detection during SLAM capture.
