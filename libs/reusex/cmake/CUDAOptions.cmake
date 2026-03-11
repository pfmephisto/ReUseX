set(CMAKE_CUDA_ARCHITECTURES 80 86 89)
#set(CMAKE_CUDA_ARCHITECTURES "native")
# RTX A6000 should have compute capability 8.6 (86)

# Recommended: only architectures you care about
# This seems to be required for PyTorch to recognize the GPU
set(TORCH_CUDA_ARCH_LIST "8.6;8.0;8.9")  # RTX A6000 and other Ampere GPUs
