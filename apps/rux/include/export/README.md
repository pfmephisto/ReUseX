
# Rhino
## Layer Structure
.
└── ReUseX-<timestamp>
    ├── cloud # include normal if available
    ├── semantic
    │   ├── wall # cloud per instance if instances are available otherwise as single blob
    │   └── ceiling
    │   └── ...
    ├── mesh # dis-joined meshes saved as individual rhino 
    ├── 360 # sphere # Include User strings
    └── materials # Save materials as Rhino dots with the properties attached as user strings

# Speckle
## Project Structure

Project # project needs to be specified for upload
├── cloud # include normal if available
├── semantic (collection)
│   ├── wall # cloud per instance if instances are available otherwise as single blob
│   └── ceiling
│   └── ...
├── mesh # volumes saved as individual meshes
├── 360 # sphere # Include User strings
└── materials # Save materials as points with all the properties as properties

