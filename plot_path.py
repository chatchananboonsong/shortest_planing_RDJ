"""
Plot Path CLI Launcher
สร้างแผนที่การเดินของหุ่นยนต์ (จาก CSV Log หรืออัลกอริทึม BFS)
"""

import os
import sys

# นำเข้าโมดูลจากไดเรกทอรี src
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from plot_path import main, plot_navigation_map_from_csv, create_path_map, plot_multi_round_comparison

if __name__ == "__main__":
    main()
