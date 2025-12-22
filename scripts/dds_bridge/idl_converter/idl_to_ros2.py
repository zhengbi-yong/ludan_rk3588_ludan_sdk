#!/usr/bin/env python3
"""
IDL到ROS2消息转换器
直接从IDL文件生成ROS2消息定义

Usage:
    python3 idl_to_ros2.py --idl /path/to/file.idl --output /path/to/output
"""

import argparse
import os
import re
import sys
from pathlib import Path

class IDLToROS2Converter:
    def __init__(self):
        self.ros2_types = {
            'uint8': 'uint8',
            'uint16': 'uint16',
            'uint32': 'uint32',
            'uint64': 'uint64',
            'int8': 'int8',
            'int16': 'int16',
            'int32': 'int32',
            'int64': 'int64',
            'float': 'float32',
            'double': 'float64',
            'boolean': 'bool',
            'char': 'char',
            'string': 'string',
            'octet': 'byte'
        }

    def parse_idl(self, idl_file):
        """解析IDL文件"""
        with open(idl_file, 'r') as f:
            content = f.read()

        # 移除注释
        content = re.sub(r'//.*?\n', '\n', content)
        content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)

        # 解析模块
        module_match = re.search(r'module\s+(\w+)\s*{', content)
        module_name = module_match.group(1) if module_match else 'unknown'

        # 解析常量
        constants = {}
        for const_match in re.finditer(r'const\s+(\w+)\s+(\w+)\s*=\s*(\d+);', content):
            const_type, const_name, const_value = const_match.groups()
            constants[const_name] = (const_type, const_value)

        # 解析结构体
        structs = {}
        struct_pattern = r'struct\s+(\w+)\s*{(.*?)?};'
        for struct_match in re.finditer(struct_pattern, content, re.DOTALL):
            struct_name = struct_match.group(1)
            struct_body = struct_match.group(2) or ''

            fields = []
            for field_match in re.finditer(r'(\w+(?:\[\d*\])?)\s+(\w+);', struct_body):
                field_type, field_name = field_match.groups()

                # 处理数组
                array_size = None
                if '[' in field_type and ']' in field_type:
                    type_match = re.match(r'(\w+)\[(\d*)\]', field_type)
                    if type_match:
                        base_type = type_match.group(1)
                        size_str = type_match.group(2)
                        array_size = int(constants.get(size_str, size_str)) if size_str else None
                        field_type = base_type

                # 转换类型
                ros2_type = self.ros2_types.get(field_type, field_type)
                if array_size:
                    ros2_type = f'{ros2_type}[{array_size}]'

                # 提取注释
                comment = ''
                comment_match = re.search(rf'{field_match.group(0)}\s*//\s*(.*)', struct_body)
                if comment_match:
                    comment = comment_match.group(1).strip()

                fields.append({
                    'name': field_name,
                    'type': ros2_type,
                    'comment': comment
                })

            structs[struct_name] = {
                'fields': fields,
                'constants': constants
            }

        return {
            'module': module_name,
            'constants': constants,
            'structs': structs
        }

    def generate_ros2_msg(self, struct_name, struct_data, module_name):
        """生成ROS2消息文件"""
        msg_content = []

        # 添加头部注释
        msg_content.append(f"# ROS2 message generated from {module_name} IDL")
        msg_content.append(f"# Source struct: {struct_name}")
        msg_content.append("")

        # 添加字段
        for field in struct_data['fields']:
            if field['comment']:
                msg_content.append(f"  # {field['comment']}")
            msg_content.append(f"{field['type']} {field['name']}")
            msg_content.append("")

        return '\n'.join(msg_content).strip()

    def generate_package_xml(self, package_name):
        """生成package.xml"""
        return f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>1.0.0</version>
  <description>ROS2 messages generated from IDL</description>
  <maintainer email="auto@example.com">auto-generated</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>'''

    def generate_cmakelists(self, package_name):
        """生成CMakeLists.txt"""
        msg_files = []
        for struct_name in self.parsed_data['structs']:
            msg_files.append(f'  "msg/{struct_name}.msg"')

        return f'''cmake_minimum_required(VERSION 3.5)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces({{PROJECT_NAME}}
{',\\n'.join(msg_files)}
)

ament_package()'''

    def convert_idl_to_ros2(self, idl_file, output_dir):
        """转换IDL文件到ROS2包"""
        # 解析IDL
        self.parsed_data = self.parse_idl(idl_file)

        # 创建包目录结构
        module_name = self.parsed_data['module']
        package_name = f"{module_name}_ros2"

        os.makedirs(os.path.join(output_dir, package_name, 'msg'), exist_ok=True)

        # 生成消息文件
        for struct_name, struct_data in self.parsed_data['structs'].items():
            msg_content = self.generate_ros2_msg(struct_name, struct_data, module_name)
            msg_file = os.path.join(output_dir, package_name, 'msg', f"{struct_name}.msg")
            with open(msg_file, 'w') as f:
                f.write(msg_content)
            print(f"✓ 生成消息文件: {msg_file}")

        # 生成package.xml
        package_xml = self.generate_package_xml(package_name)
        package_xml_file = os.path.join(output_dir, package_name, 'package.xml')
        with open(package_xml_file, 'w') as f:
            f.write(package_xml)
        print(f"✓ 生成package.xml: {package_xml_file}")

        # 生成CMakeLists.txt
        cmakelists = self.generate_cmakelists(package_name)
        cmakelists_file = os.path.join(output_dir, package_name, 'CMakeLists.txt')
        with open(cmakelists_file, 'w') as f:
            f.write(cmakelists)
        print(f"✓ 生成CMakeLists.txt: {cmakelists_file}")

        return os.path.join(output_dir, package_name)

def main():
    parser = argparse.ArgumentParser(description='IDL到ROS2消息转换器')
    parser.add_argument('--idl', required=True, help='IDL文件路径')
    parser.add_argument('--output', required=True, help='输出目录')
    parser.add_argument('--build', action='store_true', help='自动构建ROS2包')
    args = parser.parse_args()

    if not os.path.exists(args.idl):
        print(f"错误: IDL文件不存在: {args.idl}")
        sys.exit(1)

    os.makedirs(args.output, exist_ok=True)

    converter = IDLToROS2Converter()
    print(f"转换IDL文件: {args.idl}")
    print(f"输出目录: {args.output}")
    print("-" * 50)

    try:
        package_dir = converter.convert_idl_to_ros2(args.idl, args.output)
        print(f"\n✓ 转换完成! ROS2包位置: {package_dir}")

        if args.build:
            print("\n开始构建ROS2包...")
            import subprocess
            subprocess.run(['colcon', 'build', '--symlink-install'],
                         cwd=package_dir, check=True)
            print("✓ 构建完成!")

    except Exception as e:
        print(f"✗ 转换失败: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()