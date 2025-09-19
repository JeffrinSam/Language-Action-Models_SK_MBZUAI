#!/usr/bin/env python3
"""
Task Manager for Version2.py - View and Run All Tasks
====================================================

This script provides a comprehensive interface to:
- View all available tasks and their status
- Run individual tasks or all tasks
- Monitor data collection progress
- View collected data statistics

Usage:
    python3 task_manager.py                    # Interactive menu
    python3 task_manager.py --view              # View all tasks status
    python3 task_manager.py --run-all          # Run all tasks
    python3 task_manager.py --run-task [name]  # Run specific task
"""

import os
import sys
import subprocess
import argparse
import time
from pathlib import Path

# Task definitions
TASKS = {
    "pick_up": {
        "name": "Pick Up Cube",
        "description": "Grasp cube from table and lift",
        "episodes": 5,
        "folder": "pick_up"
    },
    "lift": {
        "name": "Lift Cube", 
        "description": "Lift cube to defined position",
        "episodes": 5,
        "folder": "lift"
    },
    "place_in_bin": {
        "name": "Place in Bin",
        "description": "Place cube into bin container",
        "episodes": 5,
        "folder": "place_in_bin"
    },
    "complete_pick_place": {
        "name": "Complete Pick & Place",
        "description": "Full pick and place sequence",
        "episodes": 5,
        "folder": "complete_pick_place"
    },
    "place_on_tray": {
        "name": "Place on Tray",
        "description": "Place cube on tray surface",
        "episodes": 5,
        "folder": "place_on_tray"
    },
    "stack_cube": {
        "name": "Stack Cube",
        "description": "Stack cube on top of another",
        "episodes": 5,
        "folder": "stack_cube"
    },
    "unstack_cube": {
        "name": "Unstack Cube",
        "description": "Remove cube from stack",
        "episodes": 5,
        "folder": "unstack_cube"
    },
    "open_drawer": {
        "name": "Open Drawer",
        "description": "Grasp handle and pull drawer open",
        "episodes": 5,
        "folder": "open_drawer"
    },
    "close_drawer": {
        "name": "Close Drawer",
        "description": "Grasp handle and push drawer closed",
        "episodes": 5,
        "folder": "close_drawer"
    },
    "push_button": {
        "name": "Push Button",
        "description": "Press button with precise motion",
        "episodes": 5,
        "folder": "push_button"
    }
}

def get_task_status():
    """Get status of all tasks."""
    status = {}
    base_path = Path("version2_data")
    
    for task_id, task_info in TASKS.items():
        folder_path = base_path / task_info["folder"]
        
        if folder_path.exists():
            csv_files = list(folder_path.glob("record_trail_*.csv"))
            completed = len(csv_files)
            total_size = sum(f.stat().st_size for f in csv_files) / 1024  # KB
            
            status[task_id] = {
                "completed": completed,
                "expected": task_info["episodes"],
                "size_kb": total_size,
                "files": csv_files,
                "status": "Complete" if completed == task_info["episodes"] else f"{completed}/{task_info['episodes']}"
            }
        else:
            status[task_id] = {
                "completed": 0,
                "expected": task_info["episodes"],
                "size_kb": 0,
                "files": [],
                "status": "Not Started"
            }
    
    return status

def print_task_status():
    """Print comprehensive task status."""
    print("🤖 VERSION2.PY TASK MANAGER")
    print("=" * 50)
    print()
    
    status = get_task_status()
    total_completed = 0
    total_expected = 0
    total_size = 0
    
    for i, (task_id, task_info) in enumerate(TASKS.items(), 1):
        task_status = status[task_id]
        
        # Status emoji
        if task_status["status"] == "Complete":
            status_emoji = "✅"
        elif task_status["completed"] > 0:
            status_emoji = "🔄"
        else:
            status_emoji = "⏳"
        
        print(f"{status_emoji} Task {i:2d}: {task_info['name']}")
        print(f"    📝 {task_info['description']}")
        print(f"    📊 Status: {task_status['status']}")
        print(f"    💾 Data: {task_status['size_kb']:.1f} KB")
        print()
        
        total_completed += task_status["completed"]
        total_expected += task_status["expected"]
        total_size += task_status["size_kb"]
    
    print("📈 OVERALL STATISTICS")
    print("-" * 30)
    print(f"Total Tasks: {len(TASKS)}")
    print(f"Completed Episodes: {total_completed}/{total_expected}")
    print(f"Total Data Size: {total_size:.1f} KB")
    print(f"Completion Rate: {(total_completed/total_expected)*100:.1f}%")
    print()

def run_task(task_id, speed=1.0):
    """Run a specific task."""
    if task_id not in TASKS:
        print(f"❌ Error: Task '{task_id}' not found!")
        print(f"Available tasks: {', '.join(TASKS.keys())}")
        return False
    
    task_info = TASKS[task_id]
    print(f"🚀 Starting Task: {task_info['name']}")
    print(f"📝 Description: {task_info['description']}")
    print(f"🎯 Expected Episodes: {task_info['episodes']}")
    if speed != 1.0:
        print(f"⚡ Speed: {speed}x faster")
    print()
    
    try:
        # Set environment variable for speed control
        env = os.environ.copy()
        env['TASK_SPEED_MULTIPLIER'] = str(speed)
        
        # Run the task
        cmd = ["python3", "version2.py", "--auto", "--task", task_id]
        result = subprocess.run(cmd, capture_output=True, text=True, env=env)
        
        if result.returncode == 0:
            print(f"✅ Task '{task_info['name']}' completed successfully!")
            return True
        else:
            print(f"❌ Task '{task_info['name']}' failed!")
            print(f"Error: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error running task: {e}")
        return False

def run_all_tasks(speed=1.0):
    """Run all tasks sequentially."""
    print("🚀 RUNNING ALL TASKS")
    if speed != 1.0:
        print(f"⚡ SPEED: {speed}x faster")
    print("=" * 30)
    print()
    
    successful = 0
    failed = 0
    
    for i, (task_id, task_info) in enumerate(TASKS.items(), 1):
        print(f"📋 Task {i}/{len(TASKS)}: {task_info['name']}")
        print("-" * 40)
        
        if run_task(task_id, speed):
            successful += 1
        else:
            failed += 1
        
        print()
        
        # Pause between tasks (except for the last one)
        if i < len(TASKS):
            pause_time = max(1, int(3/speed))  # Scale pause time with speed
            print(f"⏳ Pausing {pause_time} seconds before next task...")
            time.sleep(pause_time)
    
    print("🎉 ALL TASKS COMPLETED!")
    print("=" * 30)
    print(f"✅ Successful: {successful}")
    print(f"❌ Failed: {failed}")
    print(f"📊 Success Rate: {(successful/(successful+failed))*100:.1f}%")

def interactive_menu():
    """Show interactive menu."""
    while True:
        print("\n🤖 VERSION2.PY TASK MANAGER")
        print("=" * 40)
        print("1. View Task Status")
        print("2. Run Individual Task")
        print("3. Run All Tasks")
        print("4. View Data Statistics")
        print("5. Exit")
        print()
        
        choice = input("Select option (1-5): ").strip()
        
        if choice == "1":
            print_task_status()
            
        elif choice == "2":
            print("\n📋 AVAILABLE TASKS:")
            for i, (task_id, task_info) in enumerate(TASKS.items(), 1):
                print(f"{i:2d}. {task_info['name']} ({task_id})")
            
            try:
                task_num = int(input(f"\nSelect task number (1-{len(TASKS)}): "))
                if 1 <= task_num <= len(TASKS):
                    task_id = list(TASKS.keys())[task_num - 1]
                    run_task(task_id)
                else:
                    print("❌ Invalid task number!")
            except ValueError:
                print("❌ Please enter a valid number!")
                
        elif choice == "3":
            print("⚡ Speed options:")
            print("   1. Normal speed (1x)")
            print("   2. Fast speed (2x)")
            print("   3. Very fast speed (3x)")
            print("   4. Ultra fast speed (5x)")
            
            speed_choice = input("Select speed (1-4): ").strip()
            speed_map = {"1": 1.0, "2": 2.0, "3": 3.0, "4": 5.0}
            speed = speed_map.get(speed_choice, 1.0)
            
            confirm = input(f"⚠️  This will run all 10 tasks at {speed}x speed. Continue? (y/N): ").strip().lower()
            if confirm == 'y':
                run_all_tasks(speed)
            else:
                print("❌ Cancelled.")
                
        elif choice == "4":
            print_data_statistics()
            
        elif choice == "5":
            print("👋 Goodbye!")
            break
            
        else:
            print("❌ Invalid option! Please select 1-5.")

def print_data_statistics():
    """Print detailed data statistics."""
    print("\n📊 DETAILED DATA STATISTICS")
    print("=" * 40)
    
    status = get_task_status()
    base_path = Path("version2_data")
    
    if not base_path.exists():
        print("❌ No data folder found!")
        return
    
    total_files = 0
    total_size = 0
    
    for task_id, task_info in TASKS.items():
        task_status = status[task_id]
        folder_path = base_path / task_info["folder"]
        
        print(f"\n📁 {task_info['name']} ({task_id})")
        print(f"   📂 Folder: {folder_path}")
        print(f"   📊 Episodes: {task_status['completed']}/{task_status['expected']}")
        print(f"   💾 Size: {task_status['size_kb']:.1f} KB")
        
        if task_status['files']:
            print(f"   📄 Files:")
            for file in sorted(task_status['files']):
                size_kb = file.stat().st_size / 1024
                print(f"      - {file.name} ({size_kb:.1f} KB)")
        
        total_files += task_status['completed']
        total_size += task_status['size_kb']
    
    print(f"\n🎯 SUMMARY:")
    print(f"   Total CSV Files: {total_files}")
    print(f"   Total Data Size: {total_size:.1f} KB ({total_size/1024:.1f} MB)")
    print(f"   Average per Task: {total_size/len(TASKS):.1f} KB")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Task Manager for Version2.py")
    parser.add_argument("--view", action="store_true", help="View task status")
    parser.add_argument("--run-all", action="store_true", help="Run all tasks")
    parser.add_argument("--run-task", type=str, help="Run specific task")
    parser.add_argument("--stats", action="store_true", help="Show data statistics")
    parser.add_argument("--speed", type=float, default=1.0, help="Speed multiplier (1.0 = normal, 3.0 = 3x faster)")
    
    args = parser.parse_args()
    
    if args.view:
        print_task_status()
    elif args.run_all:
        run_all_tasks(args.speed)
    elif args.run_task:
        run_task(args.run_task, args.speed)
    elif args.stats:
        print_data_statistics()
    else:
        interactive_menu()

if __name__ == "__main__":
    main()
