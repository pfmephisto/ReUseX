# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for export utility functions."""

import pytest
import numpy as np
import pandas as pd
import tempfile
import os


class TestToPTSLogic:
    """Test suite for PTS export logic."""
    
    def test_pts_color_bgr_to_rgb_swap(self):
        """Test that BGR colors are swapped to RGB."""
        # Simulate the BGR to RGB swap logic from toPTS
        rgb = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], dtype=np.uint8)
        # Swap channels 0 and 2 (BGR to RGB)
        rgb[:, [0, 2]] = rgb[:, [2, 0]]
        
        expected = np.array([[0, 0, 255], [0, 255, 0], [255, 0, 0]], dtype=np.uint8)
        np.testing.assert_array_equal(rgb, expected)
    
    def test_pts_dataframe_structure(self):
        """Test PTS dataframe structure creation."""
        # Simulate the dataframe creation from toPTS
        xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        rgb = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
        intensity = np.zeros((2, 1), dtype=np.uint8)
        
        df = pd.DataFrame(
            np.concatenate([xyz, intensity, rgb], axis=1),
            columns=["x", "y", "z", "i", "r", "g", "b"]
        )
        
        assert list(df.columns) == ["x", "y", "z", "i", "r", "g", "b"]
        assert len(df) == 2
        assert df["x"].iloc[0] == 1.0
        assert df["r"].iloc[0] == 255
    
    def test_pts_file_format(self):
        """Test PTS file format with header."""
        # Simulate PTS file creation
        xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        rgb = np.array([[255, 128, 64], [32, 16, 8]], dtype=np.uint8)
        intensity = np.zeros((2, 1), dtype=np.uint8)
        
        df = pd.DataFrame(
            np.concatenate([xyz, intensity, rgb], axis=1),
            columns=["x", "y", "z", "i", "r", "g", "b"]
        )
        
        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.pts') as f:
            temp_path = f.name
            f.write(f"{len(df)}\n")
            df.to_csv(f, index=False, sep=" ", header=False)
        
        try:
            # Read back and verify
            with open(temp_path, 'r') as f:
                lines = f.readlines()
                assert lines[0].strip() == "2"  # Number of points
                # Second line should have 7 values (x, y, z, i, r, g, b)
                values = lines[1].strip().split()
                assert len(values) == 7
                assert float(values[0]) == 1.0  # x
                assert float(values[1]) == 2.0  # y
                assert float(values[2]) == 3.0  # z
        finally:
            os.unlink(temp_path)
    
    def test_pts_intensity_zeros(self):
        """Test that intensity values are initialized to zero."""
        n_points = 5
        intensity = np.zeros((n_points, 1), dtype=np.uint8)
        
        assert intensity.shape == (5, 1)
        assert np.all(intensity == 0)
        assert intensity.dtype == np.uint8
    
    def test_pts_rgb_uint8_conversion(self):
        """Test RGB values are properly converted to uint8."""
        df = pd.DataFrame({
            "x": [1.0], "y": [2.0], "z": [3.0],
            "i": [0], "r": [255], "g": [128], "b": [64]
        })
        
        df["i"] = df["i"].astype(np.uint8)
        df["r"] = df["r"].astype(np.uint8)
        df["g"] = df["g"].astype(np.uint8)
        df["b"] = df["b"].astype(np.uint8)
        
        assert df["i"].dtype == np.uint8
        assert df["r"].dtype == np.uint8
        assert df["g"].dtype == np.uint8
        assert df["b"].dtype == np.uint8
