#!/bin/bash

# Foxglove ROS2 Bridge Management Script
# Simple interface to manage the Foxglove daemon

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Show menu
show_menu() {
    echo -e "${BLUE}====================================${NC}"
    echo -e "${BLUE}    Foxglove ROS2 Bridge Manager     ${NC}"
    echo -e "${BLUE}====================================${NC}"
    echo
    echo -e "${GREEN}1)${NC} Start Foxglove Bridge"
    echo -e "${GREEN}2)${NC} Stop Foxglove Bridge"
    echo -e "${GREEN}3)${NC} Restart Foxglove Bridge"
    echo -e "${GREEN}4)${NC} Check Status"
    echo -e "${GREEN}5)${NC} View Logs"
    echo -e "${GREEN}6)${NC} List ROS2 Topics"
    echo -e "${RED}7)${NC} Exit"
    echo
    echo -e "${YELLOW}Current Status:${NC}"
    ./scripts/foxglove_ros2.sh status 2>/dev/null || echo -e "${RED}Not running${NC}"
    echo
}

# Main loop
while true; do
    show_menu
    read -p "Enter your choice [1-7]: " choice

    case $choice in
        1)
            echo -e "\n${GREEN}Starting Foxglove Bridge...${NC}"
            ./scripts/foxglove_ros2.sh start
            echo
            read -p "Press Enter to continue..."
            ;;
        2)
            echo -e "\n${RED}Stopping Foxglove Bridge...${NC}"
            ./scripts/foxglove_ros2.sh stop
            echo
            read -p "Press Enter to continue..."
            ;;
        3)
            echo -e "\n${YELLOW}Restarting Foxglove Bridge...${NC}"
            ./scripts/foxglove_ros2.sh restart
            echo
            read -p "Press Enter to continue..."
            ;;
        4)
            echo -e "\n${BLUE}Checking Status...${NC}"
            ./scripts/foxglove_ros2.sh status
            echo
            read -p "Press Enter to continue..."
            ;;
        5)
            echo -e "\n${BLUE}Showing Logs...${NC}"
            ./scripts/foxglove_ros2.sh logs
            echo
            read -p "Press Enter to continue..."
            ;;
        6)
            echo -e "\n${BLUE}Listing ROS2 Topics...${NC}"
            ./scripts/foxglove_ros2.sh topics
            echo
            read -p "Press Enter to continue..."
            ;;
        7)
            echo -e "\n${RED}Exiting...${NC}"
            exit 0
            ;;
        *)
            echo -e "\n${RED}Invalid choice. Please enter 1-7.${NC}"
            read -p "Press Enter to continue..."
            ;;
    esac
done