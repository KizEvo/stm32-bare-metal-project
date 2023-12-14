#!/usr/bin/env python
import re

def main():
    regex_pattern = r"(settable) ([\w]*)"
    string_out_vector_handlers = ""
    string_out_vector_name = ""
    # Read lines from file and format vector name
    with open("vector_handler_raw_text.txt", mode='r') as file:
        for line in file.readlines():
            regex_result = re.search(regex_pattern, line)
            string_out_vector_handlers += ("__attribute__ ((weak, alias (\"Default_Handler\"))) void {}_Handler(void);\n".format(regex_result[2]))
            string_out_vector_name += "{}_Handler,\n".format(regex_result[2])
    # Write string to vectors_handler_c.txt
    with open("vectors_handler_c.txt", mode='w') as file:
        file.writelines(string_out_vector_handlers)
        file.writelines(string_out_vector_name)

if __name__ == "__main__":
    main()