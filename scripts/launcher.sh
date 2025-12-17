#!/bin/bash
# Main launcher script for reorganized unitree_sdk2 scripts
# This script provides easy access to all functionality while maintaining backward compatibility

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Help function
show_help() {
    echo "Unitree SDK2 Scripts Launcher"
    echo "=============================="
    echo ""
    echo "This launcher provides organized access to all Unitree SDK2 scripts."
    echo "The scripts have been reorganized into logical categories while maintaining full functionality."
    echo ""
    echo "Usage: $0 [category] [script] [options]"
    echo ""
    echo "Categories:"
    echo "  lowcmd        - Low-level command processing and joint control"
    echo "  dds_bridge    - DDS communication and CAN bridge utilities"
    echo "  deployment    - Deployment and configuration scripts"
    echo "  testing       - Testing and verification utilities"
    echo "  ros_integration - ROS1 integration and bridge scripts"
    echo ""
    echo "Key LowCmd Scripts (for /lowcmd processing):"
    echo "  $0 lowcmd start_jetson_lowcmd.sh    - Start Jetson LowCmd publisher"
    echo "  $0 lowcmd jetson_lowcmd_publisher.py - Run LowCmd publisher directly"
    echo "  $0 lowcmd test_lowcmd_format.py     - Test LowCmd data format"
    echo "  $0 lowcmd python_lowcmd_builder.py  - Demo LowCmd message builder"
    echo ""
    echo "Examples:"
    echo "  # Start LowCmd publisher with default settings"
    echo "  $0 lowcmd start_jetson_lowcmd.sh"
    echo ""
    echo "  # Test LowCmd format"
    echo "  $0 lowcmd test_lowcmd_format.py"
    echo ""
    echo "  # Show DDS bridge options"
    echo "  $0 dds_bridge"
    echo ""
    echo "  # Show all available scripts"
    echo "  $0 list"
    echo ""
    echo "Legacy Compatibility:"
    echo "  For backward compatibility, you can also run scripts directly from their subdirectories:"
    echo "  - scripts/lowcmd/start_jetson_lowcmd.sh"
    echo "  - scripts/dds_bridge/can_monitor.sh"
    echo "  - etc."
}

# List all available scripts
list_scripts() {
    echo "Available Scripts by Category:"
    echo "=============================="
    echo ""

    for category in lowcmd dds_bridge deployment testing ros_integration; do
        if [ -d "$SCRIPT_DIR/$category" ]; then
            echo -e "${BLUE}$category${NC}:"
            for script in "$SCRIPT_DIR/$category"/*.sh "$SCRIPT_DIR/$category"/*.py; do
                if [ -f "$script" ]; then
                    basename_script=$(basename "$script")
                    echo "  - $basename_script"
                fi
            done
            echo ""
        fi
    done
}

# Execute script in category
execute_script() {
    local category="$1"
    local script="$2"
    shift 2

    local script_path="$SCRIPT_DIR/$category/$script"

    if [ ! -f "$script_path" ]; then
        echo -e "${RED}Error: Script not found: $script${NC}"
        echo "Available scripts in '$category':"
        ls -1 "$SCRIPT_DIR/$category" 2>/dev/null || echo "  No scripts found in this category"
        exit 1
    fi

    echo -e "${GREEN}Executing: $category/$script${NC}"
    echo "Command: $script_path $*"
    echo ""

    # Make executable and run
    chmod +x "$script_path"
    "$script_path" "$@"
}

# Main execution
main() {
    case "${1:-help}" in
        "help"|"-h"|"--help")
            show_help
            ;;
        "list"|"-l"|"--list")
            list_scripts
            ;;
        "lowcmd")
            if [ -z "$2" ]; then
                echo -e "${BLUE}LowCmd Processing Scripts:${NC}"
                ls -1 "$SCRIPT_DIR/lowcmd" | grep -E '\.(sh|py)$'
                echo ""
                echo "Use: $0 lowcmd <script> [options]"
            else
                execute_script "lowcmd" "$2" "${@:3}"
            fi
            ;;
        "dds_bridge")
            if [ -z "$2" ]; then
                echo -e "${BLUE}DDS Bridge Scripts:${NC}"
                ls -1 "$SCRIPT_DIR/dds_bridge" | grep -E '\.(sh|py)$'
                echo ""
                echo "Use: $0 dds_bridge <script> [options]"
            else
                execute_script "dds_bridge" "$2" "${@:3}"
            fi
            ;;
        "deployment")
            if [ -z "$2" ]; then
                echo -e "${BLUE}Deployment Scripts:${NC}"
                ls -1 "$SCRIPT_DIR/deployment" | grep -E '\.(sh|py)$'
                echo ""
                echo "Use: $0 deployment <script> [options]"
            else
                execute_script "deployment" "$2" "${@:3}"
            fi
            ;;
        "testing")
            if [ -z "$2" ]; then
                echo -e "${BLUE}Testing Scripts:${NC}"
                ls -1 "$SCRIPT_DIR/testing" | grep -E '\.(sh|py)$'
                echo ""
                echo "Use: $0 testing <script> [options]"
            else
                execute_script "testing" "$2" "${@:3}"
            fi
            ;;
        "ros_integration")
            if [ -z "$2" ]; then
                echo -e "${BLUE}ROS Integration Scripts:${NC}"
                ls -1 "$SCRIPT_DIR/ros_integration" | grep -E '\.(sh|py)$'
                echo ""
                echo "Use: $0 ros_integration <script> [options]"
            else
                execute_script "ros_integration" "$2" "${@:3}"
            fi
            ;;
        *)
            echo -e "${RED}Unknown category: $1${NC}"
            echo "Available categories: lowcmd, dds_bridge, deployment, testing, ros_integration"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Check if at least one argument provided
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

main "$@"