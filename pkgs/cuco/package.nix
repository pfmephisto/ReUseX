{fetchFromGitHub}:
# Pre-fetched source for NVIDIA cuCollections.
# Not built as a standalone package — provided as CPM_cuco_SOURCE
# so RAFT's CPM can do an in-tree build (cuco has always_download=true).
fetchFromGitHub {
  owner = "NVIDIA";
  repo = "cuCollections";
  rev = "6d59add35767afaf8dbc03ee52f916b00cd0fb11";
  hash = "sha256-rsSA5u+fzdV2llU1VpLAwiY4teKphV3GO3M1U2exF6c=";
}
