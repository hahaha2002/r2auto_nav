from converter import format_array
import numpy as np
import matplotlib.pyplot as plt
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from scipy import interpolate

## Set temperature thresholds
Min_threshold = 15
Max_threshold = 35

class IrSubscriber(Node):
    def __init__(self):
        super().__init__('ir_subscriber')
        
        ## Create ir subscription
        self.subscription = self.create_subscription(
            String,
            'ir_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        global temp_array
        temp_array = msg.data


def main(args=None):
    rclpy.init(args=args)
    ir_subscriber = IrSubscriber()
    pix_res = (8,8) # pixel resolution
    xx,yy = (np.linspace(0,pix_res[0],pix_res[0]),
             np.linspace(0,pix_res[1],pix_res[1])) #setup intervals of x and y axis
    zz = np.zeros(pix_res) # set array with zeros first

    # new resolution
    pix_mult = 8 # multiplier for interpolation
    interp_res = (int(pix_mult*pix_res[0]),int(pix_mult*pix_res[1]))
    grid_x,grid_y = (np.linspace(0,pix_res[0],interp_res[0]),
                     np.linspace(0,pix_res[1],interp_res[1])) #setup intervals of x and y axis
    
    # interpolation function
    def interp(z_var):
        # cubic interpolation on the image
        # at a resolution of interp_res = (pix_mult*8 x pix_mult*8)
        f = interpolate.interp2d(xx,yy,z_var,kind='cubic')
        return f(grid_x,grid_y)

    grid_z = interp(zz) # interpolated image
    print(zz)

    plt.rcParams.update({'font.size':16})
    fig_dims = (10,9) # figure size
    fig,ax = plt.subplots(figsize=fig_dims) # start figure
    fig.canvas.manager.set_window_title('AMG8833 Image Interpolation')
    im1 = ax.imshow(grid_z,vmin=Min_threshold,vmax=Max_threshold,cmap=plt.cm.RdBu_r) # plot image, with temperature bounds
    cbar = fig.colorbar(im1,fraction=0.0475,pad=0.03) # colorbar
    cbar.set_label('Temperature [C]',labelpad=10) # temp. label
    fig.canvas.draw() # draw figure
    ax_bgnd = fig.canvas.copy_from_bbox(ax.bbox) # background for speeding up runs
    fig.show() # show figure
    pix_to_read = 64 # read all 64 pixels
    
    while True:
        rclpy.spin_once(ir_subscriber)
        temp_array_formatted = format_array(temp_array)
        status,pixels = False, temp_array_formatted # read pixels with status
        if status: # if error in pixel, re-enter loop and try again
            continue
        fig.canvas.restore_region(ax_bgnd) # restore background (speeds up run)
        new_z = interp(np.reshape(pixels,pix_res)) # interpolated image
        print('Highest temperature detected = ', np.amax(new_z))
        im1.set_data(new_z) # update plot with new interpolated temps
        ax.draw_artist(im1) # draw image again
        fig.canvas.blit(ax.bbox) # blitting - for speeding up run
        fig.canvas.flush_events() # for real-time plot

if __name__ == '__main__':
    main()
