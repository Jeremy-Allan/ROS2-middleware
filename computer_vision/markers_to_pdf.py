from PIL import Image, ImageDraw, ImageFont
import os

def create_marker_pdf():
    marker_dir = "/home/jeremyallan/workspace/ros2_kortex_ws/src/repos/ROS2-middleware/computer_vision"
    marker_ids = [0, 1, 2, 10, 11, 12, 13]
    output_pdf = os.path.join(marker_dir, "aruco_markers_to_print.pdf")

    # A4 size at 300 DPI
    a4_width = 2480
    a4_height = 3508
    
    pages = []
    
    # Process in chunks of 4
    for i in range(0, len(marker_ids), 4):
        page = Image.new('RGB', (a4_width, a4_height), 'white')
        draw = ImageDraw.Draw(page)
        
        chunk = marker_ids[i:i+4]
        
        for idx, m_id in enumerate(chunk):
            img_path = os.path.join(marker_dir, f"aruco_marker_{m_id}.png")
            marker_img = Image.open(img_path).convert('RGB')
            
            # Resize marker to be approx 8cm x 8cm (945x945 pixels at 300 DPI)
            marker_size = 945
            marker_img = marker_img.resize((marker_size, marker_size), Image.NEAREST)
            
            # Calculate grid position (2x2)
            col = idx % 2
            row = idx // 2
            
            x = 200 + col * (marker_size + 200)
            y = 300 + row * (marker_size + 400)
            
            page.paste(marker_img, (x, y))
            
            # Add label
            label = f"ID: {m_id}"
            if m_id >= 10:
                label += " (Table Anchor)"
            elif m_id == 0: label += " (Apple)"
            elif m_id == 1: label += " (Banana)"
            elif m_id == 2: label += " (Tray)"
            
            # Use default font
            draw.text((x, y - 80), label, fill="black")
            
        pages.append(page)

    if pages:
        pages[0].save(output_pdf, save_all=True, append_images=pages[1:])
        print(f"Successfully created PDF: {output_pdf}")

if __name__ == "__main__":
    create_marker_pdf()
