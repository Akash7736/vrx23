#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import messagebox
from std_msgs.msg import Int16
import tkinter as tk
from tkinter import messagebox
import time

def showMessage(message, type='info', timeout=2500):
    import tkinter as tk
    from tkinter import messagebox as msgb

    root = tk.Tk()
    root.withdraw()
    try:
        root.after(timeout, root.destroy)
        if type == 'info':
            msgb.showinfo('Info', message, master=root)
        elif type == 'warning':
            msgb.showwarning('Warning', message, master=root)
        elif type == 'error':
            msgb.showerror('Error', message, master=root)
    except:
        pass

def show_warning():
    messagebox.showinfo('warning',"this is info")
    # warning_window = tk.Toplevel(root)
    # warning_window.title("Warning")
# 
    # label = tk.Label(warning_window, text="This is a warning message!")
    # label.pack(padx=10, pady=10)

    # Close the warning window after 3000 milliseconds (3 seconds)
    # warning_window.after(3000, lambda: warning_window.destroy())
class SyncBag(Node):
   def __init__(self):
      super().__init__('syncbag')
      # self.pub = self.create_publisher(ActionMesg,'/newtop',10)
      # acmsg = ActionMesg()
      # acmsg.data = [1.0,2.0,3.0]
      # self.pub.publish(acmsg)
  
      self.subscriptionbutton = self.create_subscription(
            Int16,
            '/record',
            self.reccallback,
            10)
      self.subscriptionbutton
   def reccallback(self,msg):
        
        if msg.data == 1:
            showMessage("This is warn")
        # messagebox.ABORT
           
            # time.sleep(3)

            # self.val = 1
           
        
          
# Create the main window


# Call the show_warning function immediately
# show_warning()

# Run the main loop

def main(args=None):
    

    
# Run the main loop
    
    rclpy.init(args=args)

    rs = SyncBag()
    rclpy.spin(rs)
    # root.mainloop()
    

    rs.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    # root = tk.Tk()
    # root.title("Main Window")
    main()