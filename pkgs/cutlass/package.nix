{fetchFromGitHub}:
# Pre-fetched source for NVIDIA CUTLASS.
# Not built as a standalone package — provided as CPM_cutlass_SOURCE
# so RAFT's CPM can do an in-tree build with its own cmake configuration.
fetchFromGitHub {
  owner = "NVIDIA";
  repo = "cutlass";
  rev = "v4.1.0";
  hash = "sha256-ZY+6Tg/CC6fqvU764k6QNudYDpY+s8OQklG+1aXQuns=";
}
