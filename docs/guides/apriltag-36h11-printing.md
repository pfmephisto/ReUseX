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

Enter the AprilTag development shell:

```bash
nix develop .#april
```

This provides Python (with numpy, pillow, reportlab), ImageMagick (`magick`), and `img2pdf`.

## Generating AprilTag Images

### Method 1: Python Script (Recommended)

The `tools/april/generate_apriltags.py` script embeds the official tag36h11 code data and generates correct AprilTag images directly. No external repositories or libraries needed.

```bash
# Generate tags 0-19 at 100mm @ 300dpi (default)
python tools/april/generate_apriltags.py --output tags/

# Generate specific range
python tools/april/generate_apriltags.py --start 0 --count 50 --output tags/

# Custom size (150mm @ 300dpi = 1772px)
python tools/april/generate_apriltags.py --size 1772 --border 75 --output tags/

# See all options
python tools/april/generate_apriltags.py --help
```

**Options:**
| Flag | Default | Description |
|------|---------|-------------|
| `--start` | 0 | Starting tag ID |
| `--count` | 20 | Number of tags to generate |
| `--size` | 1181 | Output image size in pixels (1181 = 100mm @ 300dpi) |
| `--border` | 50 | White quiet zone border in pixels |
| `--output` | `tags/` | Output directory |

Output files are named `tag36h11_00000.png`, `tag36h11_00001.png`, etc.

### Method 2: Official Repository + ImageMagick

Download pre-rendered tags from the official AprilTag images repository, then resize with ImageMagick using nearest-neighbor interpolation (critical for sharp edges):

```bash
# Clone official tag images
git clone https://github.com/AprilRobotics/apriltag-imgs.git
cd apriltag-imgs/tag36h11/
```

Resize to print size (100mm @ 300dpi = 1181px) with a white border:

**bash:**
```bash
mkdir -p tags_resized
for f in tag36_11_000{00..19}.png; do
    magick "$f" \
        -filter point -interpolate nearest \
        -resize 1181x1181 \
        -colorspace gray -type bilevel \
        -bordercolor white -border 50 \
        "tags_resized/$(basename "$f")"
done
```

**fish:**
```fish
mkdir -p tags_resized
for f in tag36_11_000{00..19}.png
    magick $f \
        -filter point -interpolate nearest \
        -resize 1181x1181 \
        -colorspace gray -type bilevel \
        -bordercolor white -border 50 \
        tags_resized/(basename $f)
end
```

**Key ImageMagick flags:**
- `-filter point -interpolate nearest`: Prevents blurry/grey pixels at edges
- `-colorspace gray -type bilevel`: Pure black and white output (no anti-aliasing)
- `-bordercolor white -border 50`: Adds white quiet zone around the tag

Create a PDF from the resized images:

```bash
img2pdf tags_resized/*.png -o apriltags_36h11.pdf
```

## Creating Print-Ready PDFs

### Grid Layout with ReportLab

The `tools/april/create_apriltag_pdf.py` script arranges tags in a grid on A4/letter pages with labels and crop marks.

```bash
# Default: 100mm tags, 2x3 grid on A4
python tools/april/create_apriltag_pdf.py --input tags/ --output apriltags.pdf

# Larger tags, fewer per page
python tools/april/create_apriltag_pdf.py --input tags/ --output apriltags.pdf \
    --tag-size 150 --cols 1 --rows 2

# Letter paper, no labels
python tools/april/create_apriltag_pdf.py --input tags/ --output apriltags.pdf \
    --page letter --no-labels

# See all options
python tools/april/create_apriltag_pdf.py --help
```

**Options:**
| Flag | Default | Description |
|------|---------|-------------|
| `--input` | (required) | Directory with tag PNG images |
| `--output` | `apriltags_36h11.pdf` | Output PDF file |
| `--tag-size` | 100 | Tag size in mm |
| `--cols` | 2 | Columns per page |
| `--rows` | 3 | Rows per page |
| `--page` | A4 | Page size (A4 or letter) |
| `--margin` | 20 | Page margin in mm |
| `--no-labels` | false | Omit tag ID labels |

The script accepts both official repo filenames (`tag36_11_*.png`) and generated filenames (`tag36h11_*.png`).

### Simple PDF with img2pdf

For one tag per page without grid layout:

```bash
img2pdf tags/*.png -o apriltags_36h11.pdf
```

## Complete Workflow Example

```bash
# 1. Enter devshell
nix develop .#april

# 2. Generate tags 0-19
python tools/april/generate_apriltags.py --count 20 --output tags/

# 3. Create print-ready PDF
python tools/april/create_apriltag_pdf.py --input tags/ --output apriltags.pdf --tag-size 150

# 4. Print at 100% scale
# Open apriltags.pdf and print (see Printing Best Practices below)
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
2. **Verify scale**: Measure the printed tag with a ruler - should match specified size exactly
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

### Size Calculation

To calculate pixel size for a given physical size and DPI:

```
pixels = size_mm * dpi / 25.4
```

| Physical Size | 300 DPI | 600 DPI |
|---------------|---------|---------|
| 50mm | 591 px | 1181 px |
| 100mm | 1181 px | 2362 px |
| 150mm | 1772 px | 3543 px |
| 200mm | 2362 px | 4724 px |

## Using AprilTags with ReUseX

### Integration with RTABMap SLAM

AprilTags can be detected during SLAM and used for:

1. **Loop closure**: Improve mapping accuracy
2. **Scale reference**: Provide known distances
3. **Coordinate alignment**: Establish reference frames

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

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| No detection | Low contrast | Use laser printer, check for grey pixels |
| Blurry tag edges | Wrong resize interpolation | Use `magick -filter point` or PIL `NEAREST` |
| Grey pixels in tag | Anti-aliased scaling | Add `-type bilevel` to ImageMagick or use bilevel PIL mode |
| Partial detection | Warped/curved tag | Mount on rigid flat surface |
| Intermittent detection | Reflections | Use matte paper, avoid glossy lamination |
| Wrong ID detected | Poor print quality | Increase DPI, use laser printer |
| Wrong printed size | PDF viewer scaling | Print at exactly 100%, disable "fit to page" |

## Resources

**Official AprilTag:**
- Repository: https://github.com/AprilRobotics/apriltag
- Pre-rendered images: https://github.com/AprilRobotics/apriltag-imgs
- Paper: https://april.eecs.umich.edu/software/apriltag

**ReUseX scripts:**
- Tag generation: `tools/april/generate_apriltags.py`
- PDF creation: `tools/april/create_apriltag_pdf.py`
- Dev environment: `nix develop .#april`
