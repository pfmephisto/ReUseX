# Creating a 3D Model from Scans: A Complete Walkthrough

This guide documents the complete end-to-end workflow for transforming raw SLAM scans and 360° images into semantically-rich, simplified 3D surface models using ReUseX. The resulting models are suitable for building reuse projects, architectural documentation, spatial analysis, and digital twin applications.

## Prerequisites

### Hardware Equipment

The workflow described in this guide uses an iPad Pro with LiDAR for SLAM-based spatial mapping and positioning, paired with an Insta360 X4 camera for capturing 360° panoramic imagery. This combination allows us to create both accurate geometric representations and photorealistic textures from the same capture session.

### Software Requirements

You'll need a complete ReUseX installation with all dependencies, along with RTABMap for database viewing and loop closure management. The Hugin/PanoTools suite (`pto_gen`, `pto_template`, `hugin_executor`) handles the 360° image stitching, while ExifTool verifies timestamp alignment between the scanner and camera data.

### Pre-trained Models

ReUseX leverages deep learning for semantic segmentation, requiring pre-trained models to be available before processing. The current version uses SAM3 (Segment Anything Model 3) optimized with TensorRT for universal image segmentation, providing significant performance improvements over previous versions. Earlier versions of ReUseX utilized YOLO models for architectural element detection. Model files should be placed in your project root directory or their paths specified via command-line arguments.

![Equipment Setup](./images/equipment-setup.png)
*The iPad Pro with LiDAR and Insta360 X4 used for data capture*

## Phase 1: Preparation and Time Synchronization

Before beginning any scan, the most critical preparatory step is synchronizing the timestamps between your scanner and camera. This synchronization enables accurate spatial placement of 360° images along the scanning path that will be reconstructed later. Take the time to ensure both the iPad Pro and Insta360 X4 have their clocks synchronized with minimal offset—ideally less than one second. Consider using NTP or another reference time source to achieve precise synchronization. Document the exact time sync for reference, as you'll need to verify this alignment later in the process.

## Phase 2: Initial Scanning Pass

The scanning process begins with creating a foundational spatial map of the environment using the scanner alone. Walk through the building systematically with the iPad Pro, ensuring comprehensive coverage of all areas of interest. Maintain steady, deliberate movement and proper scanner orientation throughout the pass, allowing the device to capture sufficient feature points for accurate mapping. This initial pass establishes the baseline geometry and positional framework that all subsequent steps will build upon.

![Scanning Process](./images/scanning-walkthrough.png)
*Screenshot showing the scanning interface during the walkthrough*

## Phase 3: Image Capture Pass

With the spatial map established, perform a second walkthrough—this time capturing 360° images at key locations throughout the space. The scanner should remain active during this pass, as it simultaneously records its position for each captured image. This positional data becomes crucial for geolocating the panoramic imagery within the 3D coordinate system established during the initial scan. Walk through the building with both devices, pausing to capture 360° images at locations where you want visual documentation or where complex spatial relationships need photographic reference.

![Image Capture Locations](./images/capture-locations.png)
*Visualization showing the locations where 360° images were captured*

## Phase 4: Scan Assembly

When multiple scanning sessions have been performed—perhaps covering different floors or areas of a building—these individual databases need to be merged into a single unified representation. RTABMap provides the `rtabmap-assemble` tool for this purpose:

```bash
rtabmap-assemble "./path/to/scan1.db;./path/to/scan2.db" -o ./path/to/merged.db
```

This command combines the separate scan databases while maintaining their relative positions and coordinate systems, creating a cohesive model of the entire scanned environment.

![Multiple Scans](./images/multiple-scans.png)
*Visualization showing multiple scan databases before merging*

## Phase 5: 360° Image Processing

The raw 360° images captured by the Insta360 X4 exist in a proprietary format that requires reprojection and stitching before they can be used. This process transforms the dual-lens captures into usable equirectangular panoramic images.

### Insta360 X4 Hugin Template

First, create a template file for the Insta360 X4 lens configuration. Save the following as a `.pto` file (e.g., `insta360_x4_template.pto`):

```
# hugin project file
#hugin_ptoversion 2
p f2 w5422 h2711 v360  k1 E8.07729 R0 n"TIFF_m c:LZW r:CROP"
m i0

# image lines
#-hugin  cropFactor=1
i w5888 h2944 f2 v391 Ra0 Rb0 Rc0 Rd0 Re0 Eev8.07728723345422 Er1 Eb1 r9.67415383504551 p-15.4753436799109 y-151.183643070868 TrX0 TrY0 TrZ0 Tpy0 Tpp0 j0 a0 b0 c0 d1468.6 e0 g0 t0 Va1 Vb0 Vc0 Vd0 Vx0 Vy0  S2942,5881,0,2938 Vm5 n"IMG_20260213_101653_00_001.jpg"
#-hugin  cropFactor=1
i w5888 h2944 f2 v391 Ra0 Rb0 Rc0 Rd0 Re0 Eev8.07728723345422 Er1 Eb1 r-9.67415383504551 p15.4753436799108 y28.816356929132 TrX0 TrY0 TrZ0 Tpy0 Tpp0 j0 a0 b0 c0 d-1468.6 e0 g0 t0 Va1 Vb0 Vc0 Vd0 Vx0 Vy0  S0,2941,0,2941 Vm5 n"IMG_20260213_101653_00_001.jpg"


# specify variables that should be optimized
v Ra0
v Rb0
v Rc0
v Rd0
v Re0
v Eev0
v Vb0
v Vc0
v Vd0
v Ra1
v Rb1
v Rc1
v Rd1
v Re1
v Vb1
v Vc1
v Vd1
v


# control points
c n0 N1 x3225 y2138 X2645.63237348499 Y2132.45666117389 t0
c n0 N1 x5694 y1046 X182 Y1048 t0
c n0 N1 x5760 y1435 X108 Y1441 t0
c n0 N1 x5428.99981496392 y2409.00050462385 X489.912577366332 Y2282.31091125528 t0
c n0 N1 x3537 y2460 X2408 Y2506 t0
c n0 N1 x3380 y2284 X2575 Y2326 t0
c n0 N1 x3190 y2047 X2701 Y2040 t0
c n0 N1 x3132 y1463 X2882 Y1462 t0
c n0 N1 x3071.00001051711 y1192.00000987523 X2776.17131868548 Y1198.54417522873 t0
c n0 N1 x3253 y801 X2661 Y788 t0
c n0 N1 x3450 y546 X2471 Y516 t0
c n0 N1 x5715 y1212 X130 Y1212 t0
c n0 N1 x5685 y1188 X118 Y1173 t0
c n0 N1 x5372 y636 X395 Y554 t0

#hugin_optimizeReferenceImage 1
#hugin_blender enblend
#hugin_remapper nona
#hugin_enblendOptions
#hugin_enfuseOptions
#hugin_hdrmergeOptions -m avg -c
#hugin_verdandiOptions
#hugin_edgeFillMode 0
#hugin_edgeFillKeepInput false
#hugin_outputLDRBlended true
#hugin_outputLDRLayers true
#hugin_outputLDRExposureRemapped false
#hugin_outputLDRExposureLayers false
#hugin_outputLDRExposureBlended false
#hugin_outputLDRStacks false
#hugin_outputLDRExposureLayersFused false
#hugin_outputHDRBlended false
#hugin_outputHDRLayers false
#hugin_outputHDRStacks false
#hugin_outputLayersCompression LZW
#hugin_outputImageType jpg
#hugin_outputImageTypeCompression LZW
#hugin_outputJPEGQuality 90
#hugin_outputImageTypeHDR exr
#hugin_outputImageTypeHDRCompression LZW
#hugin_outputStacksMinOverlap 0.7
#hugin_outputLayersExposureDiff 0.5
#hugin_outputRangeCompression 0
#hugin_optimizerMasterSwitch 0
#hugin_optimizerPhotoMasterSwitch 21
```

This template defines the lens parameters, distortion correction, control points, and output settings specifically calibrated for the Insta360 X4's dual-lens configuration. The control points establish the correspondence between the front and back lens images, enabling accurate stitching along the seam.

### Automated Stitching Script

The following Fish shell function automates the stitching workflow using the Hugin toolchain:

```fish
function stitch_insp_folder
    set -l template_file $argv[1] # Path to the .pto template file
    set -l folder_path $argv[2] # Path to the folder containing .insp files

    if not test -f $template_file
        echo "Template file '$template_file' does not exist."
        return 1
    end

    if not test -d $folder_path
        echo "Folder '$folder_path' does not exist."
        return 1
    end

    for img1 in $folder_path/*.insp
        set base (basename $img1 .insp)
        set out_pto $folder_path/$base.pto
        set out_templated_pto $folder_path/$base"_templated.pto"
        set prefix $folder_path/$base

        echo "Processing $base ..."

        # 1. Generate minimal .pto
        pto_gen -o $out_pto $img1 $img1

        # 2. Apply template
        pto_template --template $template_file -o $out_templated_pto $out_pto

        # 3. Stitch
        hugin_executor --stitching --prefix=$prefix $out_templated_pto

        # 4. Clean up temporary files
        echo "Cleaning up temporary files for $base ..."
        rm -f $out_pto $out_templated_pto $folder_path/$base"_template.pto"
        rm -f $folder_path/$base*.tif

        echo "Finished $base"
    end
end
```

Call this function with `stitch_insp_folder /path/to/insta360_x4_template.pto /path/to/image/folder` to process an entire folder of captures. The script generates minimal panorama project files, applies the Insta360 X4 stitching template shown above, executes the stitch, and cleans up temporary artifacts—all automatically for each image pair.

![Image Stitching Process](./images/stitching-progress.png)
*Example of raw .insp files being processed into stitched panoramas*

## Phase 6: Timestamp Verification

After processing the images, verify that they retain correct EXIF timestamps that align with the scanner's recorded trajectory. Use ExifTool to examine the timestamp metadata:

```bash
exiftool ./path/to/image.* | grep Date
```

Compare these timestamps against the scanner's database to ensure proper temporal alignment. Any significant discrepancies here indicate synchronization problems that occurred during capture and may require manual correction or recapture of affected areas.

![Timestamp Comparison](./images/timestamp-verification.png)
*Screenshot showing EXIF timestamp verification*

## Phase 7: Database Quality Control

Before proceeding with 3D reconstruction, carefully inspect the merged database to ensure loop closures have been correctly identified and the assembled point cloud exhibits accurate geometry. Open the RTABMap database viewer with `rtabmap-databaseViewer ./path/to/merged.db` and methodically review the assembled point cloud for inconsistencies or misalignments.

Pay particular attention to loop closure connections—the associations RTABMap makes when the scanner revisits previously mapped areas. Incorrect loop closures manifest as sudden spatial shifts, duplicated geometry, or warped structures. When you identify problematic closures, delete them within the viewer interface. Similarly, if you notice areas that should be connected but aren't, manually add correct loop closures to improve the reconstruction. This quality control step is crucial, as errors here propagate through all subsequent processing stages.

![Database Viewer](./images/database-viewer.png)
*RTABMap Database Viewer showing loop closures and point cloud*

![Loop Closures](./images/loop-closures.png)
*Visualization of loop closure connections in the scan data*

## Phase 8: Semantic Annotation

With a verified scan database, the next step enriches the raw geometric data with semantic understanding. ReUseX leverages SAM3 (Segment Anything Model 3) with TensorRT optimization to perform universal image segmentation, identifying and labeling architectural elements, objects, and surfaces within the scan images. The TensorRT acceleration provides significantly faster inference compared to previous implementations. Execute the annotation process with:

```bash
rux annotate -n ./path/to/model_folder/ ./path/to/database.db
```

The `-n` parameter specifies the path to your SAM3 model directory, while the final argument points to your RTABMap database. The annotation process segments images into meaningful regions, identifies architectural elements like walls, doors, windows, and furniture, then associates these semantic labels with corresponding 3D points in the point cloud. This semantic enrichment prepares the data for intelligent downstream processing—plane segmentation benefits from knowing which surfaces are walls versus furniture, and room segmentation uses semantic boundaries to inform spatial partitioning decisions.

![Semantic Annotation](./images/semantic-labels.png)
*Scan images with semantic segmentation masks applied by SAM3*

![Labeled Point Cloud](./images/labeled-point-cloud.png)
*Point cloud colored by semantic labels after SAM3 annotation*

## Phase 9: Import and Point Cloud Generation

Now import the annotated RTABMap database into ReUseX's processing pipeline to generate the comprehensive 3D point cloud that will serve as the foundation for all subsequent geometric operations:

```bash
rux import rtabmap ./path/to/database.db
```

This import process produces a dense 3D point cloud where each point carries not just position information but also surface normals (indicating local surface orientation) and semantic labels from the previous annotation step. This rich point cloud representation enables the sophisticated geometric analysis that follows.

![Generated Point Cloud](./images/point-cloud-output.png)
*The final dense point cloud with normals and labels*

## Phase 10: Plane Segmentation

The plane segmentation phase extracts and identifies planar surfaces within the point cloud using advanced geometric algorithms. This step is fundamental to understanding the architectural structure, as it identifies walls, floors, and ceilings by detecting coplanar point clusters. Execute plane segmentation with:

```bash
rux segment planes [options]
```

The underlying algorithm employs RANSAC-based plane detection with robust outlier rejection, combined with normal-based clustering for surface identification. Minimum plane size filtering removes noise and insignificant surface fragments, while the output includes both plane equations (mathematical descriptions of each planar surface) and point assignments (which points belong to which planes). This plane segmentation serves as the foundation for room segmentation and mesh generation, establishing the primary structural geometry of the scanned space.

![Plane Segmentation](./images/segmented-planes.png)
*Visualization showing segmented planar surfaces, each colored by its detected plane*

![Plane Normals](./images/plane-normals.png)
*Detected planes with their normal vectors and equations*

## Phase 11: Room Segmentation

With planes identified, ReUseX can now partition the point cloud into individual rooms or spatial volumes. This sophisticated process uses graph-based methods powered by GraphBLAS and LAGraph for efficient spatial connectivity analysis:

```bash
rux segment rooms [options]
```

The algorithm begins with plane-based partitioning, using detected walls and floors as spatial boundaries that define room extents. It then constructs a connectivity graph linking nearby points, representing spatial relationships throughout the scanned environment. Graph cut optimization analyzes this connectivity structure to identify natural room boundaries—points strongly connected to each other but weakly connected across boundaries likely represent different rooms. Finally, volume assignment allocates each point to its containing room based on the optimization results. This room segmentation enables per-room mesh generation and facilitates room-level architectural analysis, allowing you to treat each space independently for measurement, modeling, or renovation planning.

![Room Segmentation](./images/segmented-rooms.png)
*Point cloud segmented into individual rooms, each colored uniquely*

![Room Boundaries](./images/room-boundaries.png)
*Detected room boundaries shown as wall planes separating spaces*

## Phase 12: Mesh Generation

The mesh generation phase transforms the segmented point cloud into the final simplified, room-based surface mesh. ReUseX creates clean architectural meshes by fitting continuous surfaces to detected planes and room boundaries:

```bash
rux mesh [options]
```

The mesh generation process fits mesh surfaces to segmented planes, constructing geometry per room using the spatial partitioning established earlier. Topology optimization simplifies the mesh while preserving essential architectural features—eliminating unnecessary vertices while maintaining precise representations of corners, openings, and structural elements. The process also generates texture coordinates, preparing UV mapping for subsequent texture application.

The resulting mesh exhibits several key characteristics: it's simplified with significantly reduced polygon count compared to the raw point cloud, yet maintains architectural accuracy; it's structured and organized by room and architectural elements for easy manipulation; and it forms manifold, watertight geometry suitable for further processing or analysis.

![Final Mesh](./images/final-mesh.png)
*The final simplified room-based mesh model showing clean architectural geometry*

![Mesh Wireframe](./images/mesh-wireframe.png)
*Wireframe view showing the mesh topology and simplification*

## Export and Visualization

Throughout the workflow, you can visualize intermediate results using `rux view [options]`, which provides interactive 3D visualization of point clouds and meshes. When processing is complete, export your final model using `rux export [options]`. ReUseX supports multiple formats including E57 (point cloud exchange format), PCD (Point Cloud Data format), .3dm (OpenNURBS/Rhino format), and HDF5 (hierarchical data format). Future roadmap includes IFC (Industry Foundation Classes) export for direct BIM integration.

## Conclusion

This workflow demonstrates ReUseX's complete pipeline for transforming raw SLAM scans and 360° imagery into semantically-rich, simplified 3D surface models optimized for building reuse and renovation projects. The process seamlessly integrates SLAM-based spatial mapping through RTABMap, deep learning-based semantic segmentation via SAM3 with TensorRT acceleration, advanced geometric processing leveraging CGAL and PCL, graph-based room segmentation using GraphBLAS and LAGraph, and optimized mesh generation with architectural awareness.

The resulting models support diverse applications: building reuse projects benefit from accurate documentation of existing conditions for renovation planning; architects gain precise as-built records; space planning becomes data-driven with accurate measurements and spatial analysis; digital twin applications maintain synchronized representations of physical spaces; heritage preservation efforts document historical buildings with unprecedented detail; and construction teams track progress by comparing as-built versus as-planned geometry.

### Performance Considerations

Processing performance varies significantly based on hardware configuration. GPU acceleration with TensorRT provides dramatic speedups for semantic annotation—the optimized SAM3 implementation delivers significantly faster inference compared to previous YOLO-based approaches or non-optimized implementations. A CUDA-capable GPU is strongly recommended for the annotation phase. Consider batch processing when handling multiple scans, processing them in parallel to maximize hardware utilization. Save intermediate results after each phase to enable incremental workflows and avoid reprocessing if adjustments are needed. Finally, use SSD storage rather than traditional hard drives for faster data access during processing, particularly during the annotation and import phases which involve substantial random I/O.

### Troubleshooting

When encountering poor loop closures, return to the `rtabmap-databaseViewer` to review and correct the associations. Delete false positives that cause spatial distortions and manually add correct closures where the scanner obviously revisited the same location but failed to recognize it. Ensure sufficient feature overlap between scan passes—walking too quickly or with insufficient lighting can prevent reliable loop closure detection.

Segmentation issues often trace back to earlier stages. Check semantic annotation quality by reviewing the labeled images—poor lighting, motion blur, or unusual architectural elements can confuse the neural networks. Verify plane segmentation parameters match your environment's scale and characteristics, and review point cloud density and coverage to ensure sufficient data for robust segmentation.

Mesh artifacts typically result from ambiguous room boundaries or outliers in the source data. Inspect room segmentation boundaries to identify areas where the algorithm struggled to determine spatial partitioning. Adjust plane fitting tolerance if minor surface variations are being over-segmented, and check for outlier points that should have been filtered during earlier processing stages.

---

*For detailed API documentation, refer to the generated Doxygen documentation. For questions or issues, submit an issue to the [ReUseX repository](https://github.com/pfmephisto/ReUseX).*
