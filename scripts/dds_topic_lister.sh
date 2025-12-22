#!/bin/bash

# Simple DDS Topics Lister
# Uses ROS2 command line tools to monitor all DDS topics

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_banner() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}        DDS TOPICS MONITOR${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_status() {
    local status=$1
    local message=$2
    case $status in
        "INFO") echo -e "${GREEN}[INFO]${NC} $message" ;;
        "WARN") echo -e "${YELLOW}[WARN]${NC} $message" ;;
        "ERROR") echo -e "${RED}[ERROR]${NC} $message" ;;
        *) echo "[INFO] $message" ;;
    esac
}

# Check ROS2 environment
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        print_status "WARN" "ROS2 environment not sourced"
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
            print_status "INFO" "ROS2 Humble sourced"
        else
            print_status "ERROR" "ROS2 not found"
            exit 1
        fi
    fi
}

# Get topic info
get_topic_info() {
    local topic=$1

    # Get topic type
    local topic_type=$(ros2 topic info $topic 2>/dev/null | grep "Type:" | cut -d' ' -f2)

    # Get publisher count
    local pub_count=$(ros2 topic info $topic 2>/dev/null | grep "Publisher count:" | cut -d' ' -f3)

    # Get subscription count
    local sub_count=$(ros2 topic info $topic 2>/dev/null | grep "Subscription count:" | cut -d' ' -f3)

    # Try to get sample data
    local sample_data=""
    if [[ $topic_type == *"String"* ]]; then
        sample_data=$(timeout 2 ros2 topic echo $topic --once 2>/dev/null | head -20 | tr '\n' ' ')
    fi

    echo "Type: $topic_type | Pubs: $pub_count | Subs: $sub_count"
    if [ ! -z "$sample_data" ]; then
        echo "Sample: ${sample_data:0:50}..."
    fi
}

# Monitor specific topics
monitor_topics() {
    local topics=("$@")

    while true; do
        clear
        print_banner
        echo -e "\n${YELLOW}Monitoring DDS Topics (updated every 5 seconds)${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop${NC}\n"

        echo -e "${BLUE}TOPIC                           TYPE                            STATUS${NC}"
        echo "--------------------------------------------------------------------------------"

        for topic in "${topics[@]}"; do
            # Check if topic is active
            if ros2 topic info "$topic" >/dev/null 2>&1; then
                local topic_type=$(ros2 topic info $topic 2>/dev/null | grep "Type:" | cut -d' ' -f2)
                local status="ACTIVE"

                # Check if it's DDS-related
                if [[ $topic == *"dds"* ]] || [[ $topic == *"lowcmd"* ]] || [[ $topic == *"unitree"* ]]; then
                    status="${GREEN}DDS${NC}"
                elif [[ $topic_type == *"xixi"* ]]; then
                    status="${GREEN}XIXI${NC}"
                fi

                printf "%-30s %-30s %s\n" "$topic" "$topic_type" "$status"
            else
                printf "%-30s %-30s ${RED}INACTIVE${NC}\n" "$topic" "N/A"
            fi
        done

        echo -e "\n${BLUE}Detailed Information:${NC}"
        echo "----------------------------------------"

        # Show detailed info for DDS-related topics
        for topic in "${topics[@]}"; do
            if [[ $topic == *"lowcmd"* ]] || [[ $topic == *"dds"* ]]; then
                echo -e "\n${YELLOW}$topic:${NC}"
                get_topic_info "$topic"
            fi
        done

        echo -e "\n$(date '+%H:%M:%S') - Monitoring..."
        sleep 5
    done
}

# Auto-detect DDS topics
detect_topics() {
    print_status "INFO" "Detecting DDS-related topics..."

    # Get all topics
    local all_topics=$(ros2 topic list 2>/dev/null)

    local dds_topics=()
    while IFS= read -r topic; do
        if [[ $topic == *"dds"* ]] || [[ $topic == *"lowcmd"* ]] || \
           [[ $topic == *"lowstate"* ]] || [[ $topic == *"unitree"* ]] || \
           [[ $topic == *"xixi"* ]]; then
            dds_topics+=("$topic")
        fi
    done <<< "$all_topics"

    if [ ${#dds_topics[@]} -eq 0 ]; then
        print_status "WARN" "No DDS topics found. Using common Unitree topics..."
        dds_topics=(
            "/lowcmd"
            "/lowstate"
            "/bms_cmd"
            "/bms_state"
            "/sport_mode"
            "/imu_state"
        )
    fi

    print_status "INFO" "Found ${#dds_topics[@]} DDS-related topics"
    monitor_topics "${dds_topics[@]}"
}

# Main execution
main() {
    print_banner
    check_ros2

    if [ $# -eq 0 ]; then
        # Auto-detect topics
        detect_topics
    else
        # Monitor specific topics
        print_status "INFO" "Monitoring specified topics: $*"
        monitor_topics "$@"
    fi
}

# Run main function
main "$@"