import serial # recieve COM port serial data from device
import cv2 # openCV for interpretting received matrix data as renderable image
import matplotlib.pyplot as plt # plot graph data; requires pip install matplotlib
from matplotlib.animation import FuncAnimation # attempt at getting an interractive graph that can be moved around while updating..TBD proper implementation
import numpy as np # maths and matrix library
import threading # parallel execution for reading COM serial and processing queue tasks
import queue # queue inbound tasks based on received serial data
import sys # receive execution arguments, eg for COM port selection

# Global variables
_DEBUG_RECEIVE = True #False # debug flag, set false to turn off debug

mtx_data_queue = queue.Queue()  # Queue for passing matrix data between threads
mtx_displayed = {}  # Keep track of displayed matrices and their window names

def receive_matrix_data(port):
    ser = serial.Serial(port, 115200) #begin serial on port @baudRate
    while True:
        line = ser.readline().strip()  # Read binary data
        # check for comms convention tag
        if line.startswith(b'~txBEG~'):
            print("Receiving matrix...")
            metadata = line.split(b'{')[1].split(b'}')[0]
            meta_dict = {}
            # extract matrix metadata from transmission
            for pair in metadata.split(b','):
                key, value = pair.split(b':')
                meta_dict[key.strip()] = value.strip()
            # print("Metadata:", meta_dict)      
            mtxID = meta_dict[b'ID']
            rows = int(meta_dict[b'Y'])
            cols = int(meta_dict[b'X'])
            dtype = meta_dict[b'dt']

            print("metadata:{{ID:{} {}cols X {}rows of type {}}}".format(mtxID, cols, rows, dtype))
            # stage serial matrix data, also print data to terminal if debug flag is set
            matrix_data = b""
            while True:
                line = ser.readline().strip()
                if _DEBUG_RECEIVE:
                    print("Received line:", line)  
                if line == b"~txEND~" or line == b"~txERR~":
                    break
                matrix_data += line
            # split by rows and construct matrix from recieved data
            matrix_flat = matrix_data.split(b';') 
            matrix = np.zeros((rows, cols), dtype=np.float32) # if dtype == b'f' else np.float32)
            for i, row in enumerate(matrix_flat):
                if row.strip() != b'':
                    row_data = row.split(b',')
                    for j, val in enumerate(row_data):
                        try:
                            matrix[i, j] = float(val) # if dtype == b'f' else float(val)
                        except ValueError as e:
                            print("Error: Matrix transfer interrupted or invalid matrix datatype:", val)

            mtx_data_queue.put((mtxID, matrix))  # Put the received matrix in the queue

        elif line == b'~txERR~':
            print("Error: Matrix transfer interrupted or invalid, received ~txERR~ tag from sender.")
        elif line == b'~txEND~':
            if _DEBUG_RECEIVE:
                print("End of matrix transfer tag ~txEND~ recieved without ~txBEG~ begin tag.")
        else:
            # Pass through any other received serial commands directly to terminal
            print("DEVICE SAYS:", line)

# render matrix to openCV window; each unique ID received is rendered to a new window, if the same ID is seen again the existing window is updated instead
def display_matrix(matrix_id, matrix):
    try:
        min_val = np.min(matrix)
        max_val = np.max(matrix)
        if min_val < 0 or max_val > 255:
            scaled_matrix = cv2.normalize(matrix, None, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        else:
            scaled_matrix = matrix.astype(np.uint8)
        # <<IMAGE COLOUR OPTIONS>>
        # COLORMAP_AUTUMN red to yellow
        # COLORMAP_BONE  black to white
        # COLORMAP_JET blue to green to red
        # COLORMAP_WINTER  blue to cyan
        # COLORMAP_RAINBOW  spectrum
        # COLORMAP_OCEAN  white to blue
        # COLORMAP_SUMMER  green to yellow
        img0 = cv2.applyColorMap(scaled_matrix, cv2.COLORMAP_JET)
        
        # TBD flips REMOVE, doing here as easier than micro due memory
        # Perform horizontal and vertical flip (both axes)
        img = cv2.flip(img0, -1)
        
        # target px width for rescale
        targetWidth_px = 640 
        scaleFactor = targetWidth_px / img.shape[1] # scale factor required to achieve target
        # calculate new dimensions
        width = int(img.shape[1] * scaleFactor) # target 640 width
        height = int(img.shape[0] * scaleFactor)
        resize_dim = (width, height)
        img_resize = cv2.resize(img, resize_dim, interpolation = cv2.INTER_NEAREST)

        # mtxID_str = mtxID.decode('utf-8')  # Decode byte string to regular string
        if matrix_id in mtx_displayed:
            cv2.imshow(mtx_displayed[matrix_id], img_resize)
        else:
            cv2.imshow(matrix_id, img_resize)
            mtx_displayed[matrix_id] = matrix_id
        cv2.waitKey(1)
    except queue.Empty:
        pass

# init graph with axis range bounds and render position on screen
def init_plot(x_range=(-160, 160), y_range=(-120, 120), position=(0, 550)): #ORIGINAL PIXELS: (x_range=(0, 32), y_range=(24, 0)):
    # init the plot using matplotlib
    fig, ax = plt.subplots(figsize=(8, 6))  # figsize=(8, 6) fig is the container, ax is the plot area

    # Create empty lines for original and adjusted coordinates
    original_line, = ax.plot([], [], label='EdgeCoord', marker='o', linestyle='-', color='blue')
    adjusted_line, = ax.plot([], [], label='Adjusted', marker='x', linestyle='--', color='red', markersize = 8)
    # original_line, = ax.plot(
        # x_values, y_values, 
        # label='lineName', 
        # marker='^', 
        # linestyle='--', 
        # color='red', 
        # linewidth=2, 
        # markersize=8, 
        # markerfacecolor='yellow', 
        # markeredgecolor='black'
    # )
    # Set labels, title, and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('EdgeCoord and Adjustment')
    ax.legend()

    # Set fixed axis limits for the graph
    ax.set_xlim(x_range)  # Set the range for the X-axis
    ax.set_ylim(y_range)  # Set the range for the Y-axis
    
    # Move the plot window to the desired position
    fig.canvas.manager.window.wm_geometry(f"+{position[0]}+{position[1]}")
    
    return fig, ax, original_line, adjusted_line

# update existing plot
def update_plot(original_line, adjusted_line, matrix):

    if matrix is not None:
        # Ensure that the matrix has the correct shape
        if matrix.shape[1] < 4:
            raise ValueError("Matrix must have at least 4 columns representing [x, y, x_adjusted, y_adjusted]")
            
        # Use the staged data to update the plot
        original_line.set_data(matrix[:, 0], matrix[:, 1])  # Update original coordinates (x, y)
        adjusted_line.set_data(matrix[:, 2], matrix[:, 3])  # Update adjusted coordinates (x_adj, y_adj)

        # Redraw the plot
        plt.draw()  # Redraw the figure (includes ax, lines, etc.)
        plt.pause(0.0001)  # Allow the plot to refresh <<<### TBD THIS CAUSING ISSUES WHEN USER MOVEMENT OF GRAPH WINDOW ###
    

# accepts port name to link program to device on "COMx"
def main(port):

    # Start receiving matrix data in a separate thread
    receive_thread = threading.Thread(target=receive_matrix_data, args=(port,), daemon=True)
    receive_thread.start()
    
    # Init the plot to display graph data
    fig, ax, original_line, adjusted_line = init_plot()
    
    # Staging variable to hold the latest coordinates for graph plot ani
    graph_staging_matrix = None  # init set to None before update
    
    while True:
        try:
            #mtxID, matrix = mtx_data_queue.get(timeout=1)  # Get the oldest matrix from the queue
            mtxID, matrix = mtx_data_queue.get_nowait()  # Get data without blocking
            mtxID_str = mtxID.decode('utf-8')  # Decode byte string to regular string
            #check if special tag for graph data was sent
            if mtxID_str == "graph":
                # # Update the plot with new matrix data
                graph_staging_matrix = matrix
                update_plot(original_line, adjusted_line, graph_staging_matrix)
            else:
                display_matrix(mtxID_str, matrix)
        
            mtx_data_queue.task_done()  # Mark the item as processed
        except queue.Empty:
            pass

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Close all OpenCV windows
    cv2.destroyAllWindows()
    plt.close(fig)  # Close the plot window when done

# accept COM port to listen on from user terminal, either with or without the COM prefix
if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
        # Check if "COM" prefix was provided
        if port[:3].upper() != "COM":
            # Add "COM" prefix
            port = "COM" + port
    else:
        # Set default COM port if one was not provided at code execution
        port = "COM3"
    main(port)