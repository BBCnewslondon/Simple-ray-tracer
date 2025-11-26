import sys
from PIL import Image
import os

def convert_ppm_to_png(input_ppm, output_png):
    try:
        with Image.open(input_ppm) as im:
            im.save(output_png)
        print(f"Successfully converted {input_ppm} to {output_png}")
    except FileNotFoundError:
        print(f"Error: The file {input_ppm} was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 2:
        input_file = sys.argv[1]
        output_file = sys.argv[2]
    else:
        # Default values if arguments are not provided
        input_file = "build/image.ppm"
        output_file = "image.png"
        
        # Check if default input exists in current dir if not found in build
        if not os.path.exists(input_file) and os.path.exists("image.ppm"):
             input_file = "image.ppm"

    convert_ppm_to_png(input_file, output_file)
