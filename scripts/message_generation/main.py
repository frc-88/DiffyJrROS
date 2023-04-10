import os

from std_msgs.msg import Float32MultiArray
from tj2_interfaces.msg import GameObjectsStamped, TestStaticArray
from visualization_msgs.msg import MarkerArray

from message_conversion.java_class_spec import JavaClassSpec
from message_conversion.generate_spec import filter_unique_objects, java_class_spec_generator
from message_conversion.code_artifacts import generate_java_code_from_spec, generate_type_mapping_java_code, generate_message_interface_java_code


def write_java_file(root_path: str, relative_path: str, code: str) -> None:
    abs_path = os.path.join(root_path, relative_path)
    abs_dir = os.path.dirname(abs_path)
    if not os.path.isdir(abs_dir):
        os.makedirs(abs_dir)
    print(f"Writing to {abs_path}")
    with open(abs_path, 'w') as file:
        file.write(code)


def generate_from_messages(root_path, messages):
    unique_objects = {}
    for msg in messages:
        class_spec = JavaClassSpec(msg._type)
        java_class_spec_generator(class_spec, msg)
        filter_unique_objects(unique_objects, class_spec)
        for spec in unique_objects.values():
            relative_path, code = generate_java_code_from_spec(root_path, spec)
            write_java_file(root_path, relative_path, code)
    return unique_objects


def main():
    java_root = "../../src/main/java"
    os.chdir(java_root)
    root_path = "frc/robot/ros/messages"
    unique_objects = generate_from_messages(
        root_path, (GameObjectsStamped(), MarkerArray(), Float32MultiArray(), TestStaticArray()))

    write_java_file(root_path, "TypeMapping.java", generate_type_mapping_java_code(
        root_path, unique_objects.values()))
    write_java_file(root_path, "RosMessage.java",
                    generate_message_interface_java_code(root_path))


if __name__ == '__main__':
    main()
