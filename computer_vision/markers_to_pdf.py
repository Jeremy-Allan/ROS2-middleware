from PIL import Image, ImageDraw, ImageFont
import os

def create_marker_pdf():
    marker_dir = "/home/jeremyallan/workspace/ros2_kortex_ws/src/repos/ROS2-middleware/computer_vision"
    # Mapping ID to Label and Position
    MARKER_DETAILS = {
        0: "ID: 0 - Apple (Object)",
        1: "ID: 1 - Banana (Object)",
        2: "ID: 2 - Tray (Object)",
        10: "ID: 10 - Front-Left corner (+X, +Y)",
        11: "ID: 11 - Front-Right corner (+X, -Y)",
        12: "ID: 12 - Back-Right corner (-X, -Y)",
        13: "ID: 13 - Back-Left corner (-X, +Y)"
    }
    
    marker_ids = list(MARKER_DETAILS.keys())
    output_pdf = os.path.join(marker_dir, "aruco_markers_to_print.pdf")

    # A4 size at 300 DPI
    a4_width = 2480
    a4_height = 3508
    
    # 5cm x 5cm at 300 DPI (approx 590 pixels)
    marker_size = 590
    
    pages = []
    
    # Process in chunks of 6 (3x2 grid) to save paper while keeping them small
    for i in range(0, len(marker_ids), 6):
        page = Image.new('RGB', (a4_width, a4_height), 'white')
        draw = ImageDraw.Draw(page)
        
        chunk = marker_ids[i:i+6]
        
        for idx, m_id in enumerate(chunk):
            img_path = os.path.join(marker_dir, f"aruco_marker_{m_id}.png")
            marker_img = Image.open(img_path).convert('RGB')
            
            # Resize
            marker_img = marker_img.resize((marker_size, marker_size), Image.NEAREST)
            
            # Calculate grid position (3 rows, 2 columns)
            col = idx % 2
            row = idx // 2
            
            x = 300 + col * (marker_size + 500)
            y = 400 + row * (marker_size + 400)
            
            page.paste(marker_img, (x, y))
            
            # Add label
            label = MARKER_DETAILS[m_id]
            draw.text((x, y - 60), label, fill="black")
            
        pages.append(page)

    if pages:
        pages[0].save(output_pdf, save_all=True, append_images=pages[1:])
        print(f"Successfully created PDF with 5cm markers: {output_pdf}")

if __name__ == "__main__":
    create_marker_pdf()
