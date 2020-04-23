""" Functions to simulate ROSBot along a track """


import cv2
import os
import numpy as np
from math import cos, sin, pi

#Image size
IMG_SIZE=(320,240)
IMG_SIZE_TRACK=(1300,1300)

#Camera
FOV=90.0 #degrees
Y_HORIZON=120
CAMERA_TILT=40 #degrees
CAMERA_HEIGHT=0.3

#Colors
COLORS={
    'floor': (96,53,6),
    'walls': (108,113,111),
    'track': (182,168,158),
    'furniture': (6,12,13)
}

#Background rectangles
BG_DATA=[[-10,10,25], [40,50,20], [60,65,40], [85,100,10], [120,150,15], [180,205,30], [250,260,5], [280, 320, 37]]

def img_condition(img):
    return np.maximum(np.minimum(img,255),0).astype(np.uint8)

def img_track():
    # Get file name
    script_dir=os.path.dirname(__file__)
    filename=os.path.join(script_dir,'..','data','race_track.png')
    img=cv2.imread(filename)
    if img is None:
        print 'race_track.png not found'

    # Rotate to get the right alignment for yaw=0
    img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Add some texture to make it a little more realistic
    sin_signal=(1+0.1*np.reshape(np.sin(np.linspace(0,1299*2.,1299)),(1299,1,1)))
    img=img_condition(sin_signal*img.astype(np.float64))
    return img


IMG_TRACK=img_track()

def deg2deg(angle,yaw):
    """ Return value of angle, angle+360, angle-360 that is closest to yaw """
    angle_replica=[angle,angle+360,angle-360]
    idx_min=np.argmin([abs(a-yaw) for a in angle_replica])
    return angle_replica[idx_min]

def background_deg2px(angle,yaw):
    return (deg_clamp_fov(angle-yaw)/FOV+0.5)*IMG_SIZE[0]

def background_rectangle2px(r_start,r_end,yaw):
    r_start=deg2deg(r_start,yaw)
    r_end=deg2deg(r_end,yaw)
    x_start=int(background_deg2px(r_start,yaw))
    x_end=int(background_deg2px(r_end,yaw))

    flag_visible=r_end-r_start>0

    return flag_visible, x_start, x_end


def deg_clamp_fov(v):
    """Clamp a value between -FOV/2 and FOV/2"""
    return max(-FOV/2,min(FOV/2,v))

def background(yaw,img=None):
    yaw_deg=np.rad2deg(yaw)
    w=IMG_SIZE[0]
    h=IMG_SIZE[1]
    if img is None:
        img=np.zeros((h,w,3),dtype=np.uint8)

    cv2.rectangle(img,(0,Y_HORIZON),(w,0),COLORS['walls'],-1)

    for r_start,r_end,r_height in BG_DATA:
        flag_visible, x_start, x_end = background_rectangle2px(r_start,r_end,yaw)
        if flag_visible:
            cv2.rectangle(img,(x_start,Y_HORIZON),(x_end,Y_HORIZON-r_height),COLORS['furniture'],-1)

    cv2.rectangle(img,(0,h),(w,Y_HORIZON),COLORS['floor'],-1)

    return img

def rot_z(a):
    R=np.array([[cos(a),sin(a),0],[-sin(a),cos(a),0],[0,0,1]])
    return R

def rot_x(a):
    R=np.array([[1,0,0],[0,cos(a),sin(a)],[0,-sin(a),cos(a)]])
    return R

def pose_reference():
    R=rot_x(pi)
    T=np.array([[0],[0],[1]])
    return R,T

def homography(x,y,yaw):
    """
    Scale the original image to fit our desired arena size (10 x 10)
    """
    scale=5.0
    K_reference=calibration((IMG_SIZE_TRACK[0],IMG_SIZE_TRACK[1])).dot(calibration_scale(1./scale))
    """
    Due to the fact that the track image is square but the camera image is not,
    we need some offset to center the track at coordinates x=0,y=0
    """
    x_offset=(IMG_SIZE[1]-IMG_SIZE[0])/2.
    K_camera=calibration_translation((0,x_offset)).dot(calibration((320,320)))
    R_plane,T_plane=pose_reference()
    T_camera=np.array([[x],[y],[CAMERA_HEIGHT]])
    n=np.array([[0],[0],[1]])

    R_x=rot_x(pi-np.deg2rad(CAMERA_TILT)).transpose()
    R_z=rot_z(yaw).transpose()
    H_0=R_plane+(T_plane-T_camera)*n.transpose()
    H=K_camera.dot(R_x).dot(R_z).dot(H_0).dot(np.linalg.inv(K_reference))

    return H

def calibration(img_size):
    w,h=img_size
    return np.array([[w/2.,0,w/2.],[0,h/2.,h/2.],[0,0,1]])

def calibration_translation(xy):
    x,y=xy
    return np.array([[1.,0.,x],[0.,1.,y],[0,0,1]])

def calibration_scale(scale):
    return np.diag([scale,scale,1])

def camera_image(x,y,yaw):
    H=homography(x,y,yaw)
    bg_color=(95, 56, 33) #(img[0,0,0],img[0,0,1],img[0,0,2])
    img_camera = cv2.warpPerspective(IMG_TRACK, H, (320,240), borderValue=bg_color)
    return img_camera


def test(x=0,y=0,yaw_deg=0):
    yaw=np.deg2rad(yaw_deg)

    img_camera=camera_image(x,y,yaw)

    cv2.imshow('camera',img_camera)
    cv2.waitKey(100)
