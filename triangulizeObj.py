input_file = "input.obj"
out_file = "output.obj"

output_buffer = ""

with open(input_file, 'r') as file:
    for line in file:
        if line.startswith("f "):
            components = line.split()[1:]
            if len(components) == 3:
                output_buffer += line
            else:
                # Triangulate the polygon using triangular fan method
                for i in range(1, len(components) - 1):
                    output_buffer += f"f {components[0]} {components[i]} {components[i+1]}\n"
        else: 
            output_buffer += line

with open(out_file, 'w') as file:
    file.write(output_buffer)