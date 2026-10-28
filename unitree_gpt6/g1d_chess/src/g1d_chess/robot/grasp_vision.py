"""Conservative, explicitly configured evidence for one vertical pawn grasp view.

This is not a general grasp detector. Missing or conflicting evidence fails closed.
"""
import copy
import hashlib
from pathlib import Path

import cv2
import numpy as np


def red_mask(image):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    return (((hsv[:,:,0]<=12)|(hsv[:,:,0]>=165)) &
            (hsv[:,:,1]>=70)&(hsv[:,:,2]>=45)).astype(np.uint8)*255


def _roi(value,width,height,name):
    if (not isinstance(value,list) or len(value)!=4 or
            any(isinstance(v,bool) or not isinstance(v,int) for v in value)):
        raise ValueError(f"{name} requires integer [x0,y0,x1,y1]")
    x0,y0,x1,y1=value
    if not 0<=x0<x1<=width or not 0<=y0<y1<=height:
        raise ValueError(f"{name} is outside image bounds")
    return value


def prepare_vision(config):
    config=copy.deepcopy(config)
    if config.get("camera") not in ("cam_left_high","cam_left_wrist","cam_right_wrist"):
        raise ValueError("auto gate needs an explicit supported camera")
    size=config.get("image_size")
    if not isinstance(size,list) or len(size)!=2 or any(not isinstance(v,int) or v<1 for v in size):
        raise ValueError("image_size requires [width,height]")
    for key in ("source_roi","jaw_roi"):
        _roi(config[key],*size,key)
    path=Path(config["template_path"]).resolve()
    image=cv2.imread(str(path))
    if image is None:
        raise ValueError("auto gate template image cannot be read")
    roi=_roi(config["template_roi"],image.shape[1],image.shape[0],"template_roi")
    x0,y0,x1,y1=roi
    template=red_mask(image[y0:y1,x0:x1])
    if np.count_nonzero(template)<100:
        raise ValueError("auto gate template lacks enough red pawn evidence")
    defaults={"max_source_red_pixels":30,"min_source_visible_fraction":.5,
              "min_jaw_red_pixels":200,"min_jaw_red_center_y":250.,
              "min_template_score":.70,"template_scales":[.8,.9,1.,1.1],
              "circle_radius_range":[30,65],"min_circle_red_fraction":.04}
    for key,value in defaults.items():
        config.setdefault(key,value)
    for key in ("max_source_red_pixels","min_source_visible_fraction","min_jaw_red_pixels",
                "min_jaw_red_center_y","min_template_score","min_circle_red_fraction"):
        value=config[key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value) or value<0:
            raise ValueError(f"invalid visual threshold {key}")
    if config["max_source_red_pixels"]>50 or config["min_jaw_red_pixels"]<100 or config["min_template_score"]<.5:
        raise ValueError("auto gate thresholds are too permissive")
    if not .3<=config["min_source_visible_fraction"]<=1 or not .02<=config["min_circle_red_fraction"]<=.3:
        raise ValueError("invalid visual visibility threshold")
    if not config["jaw_roi"][1]<config["min_jaw_red_center_y"]<config["jaw_roi"][3]:
        raise ValueError("jaw red center threshold must lie inside jaw ROI")
    if (not isinstance(config["template_scales"],list) or not 1<=len(config["template_scales"])<=8 or
            any(not isinstance(s,(int,float)) or not np.isfinite(s) or not .5<=s<=1.5 for s in config["template_scales"])):
        raise ValueError("invalid template scales")
    radii=config["circle_radius_range"]
    if not isinstance(radii,list) or len(radii)!=2 or not all(isinstance(r,int) for r in radii) or not 5<=radii[0]<radii[1]<=100:
        raise ValueError("invalid circle radius range")
    config.update(template_path=str(path),template_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  template_mask=template)
    return config


def evaluate_image(image,config):
    result={"passed":False,"reason":"image_missing"}
    if image is None:
        return result
    if [image.shape[1],image.shape[0]]!=config["image_size"]:
        return dict(result,reason="unexpected_image_size")
    mask=red_mask(image)
    sx0,sy0,sx1,sy1=config["source_roi"]
    source=mask[sy0:sy1,sx0:sx1]
    source_red=int(np.count_nonzero(source))
    source_gray=cv2.cvtColor(image[sy0:sy1,sx0:sx1],cv2.COLOR_BGR2GRAY)
    visible=float(np.mean(source_gray>80))
    result.update(source_red_pixels=source_red,source_visible_fraction=visible)
    if source_red>config["max_source_red_pixels"]:
        return dict(result,reason="red_piece_remains_at_source")
    if visible<config["min_source_visible_fraction"]:
        return dict(result,reason="source_not_visible")
    x0,y0,x1,y1=config["jaw_roi"]
    jaw_mask=mask[y0:y1,x0:x1]
    yy,xx=np.nonzero(jaw_mask)
    count=int(len(xx))
    center_y=float(np.mean(yy)+y0) if count else None
    result.update(jaw_red_pixels=count,jaw_red_center_y=center_y)
    if count<config["min_jaw_red_pixels"] or center_y<config["min_jaw_red_center_y"]:
        return dict(result,reason="no_red_piece_between_jaws")
    template=np.asarray(config["template_mask"],dtype=np.uint8)
    score=-1.
    for scale in config["template_scales"]:
        scaled=cv2.resize(template,None,fx=scale,fy=scale,interpolation=cv2.INTER_NEAREST)
        if scaled.shape[0]>jaw_mask.shape[0] or scaled.shape[1]>jaw_mask.shape[1]:
            continue
        score=max(score,float(cv2.minMaxLoc(cv2.matchTemplate(jaw_mask,scaled,cv2.TM_CCOEFF_NORMED))[1]))
    result["template_score"]=score
    if score<config["min_template_score"]:
        return dict(result,reason="held_red_pattern_not_confirmed")
    gray=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY)
    circles=cv2.HoughCircles(cv2.GaussianBlur(gray,(5,5),1),cv2.HOUGH_GRADIENT,1,25,
                            param1=80,param2=25,minRadius=config["circle_radius_range"][0],
                            maxRadius=config["circle_radius_range"][1])
    accepted=[]
    jaw_hsv=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
    grid_y,grid_x=np.indices(gray.shape)
    if circles is not None:
        for cx,cy,radius in circles[0]:
            inside=(xx-cx)**2+(yy-cy)**2<(.85*radius)**2
            enclosed=int(np.count_nonzero(inside))
            fraction=float(enclosed/(np.pi*radius*radius))
            disk=(grid_x-cx)**2+(grid_y-cy)**2<(.75*radius)**2
            pale=float(np.mean(((jaw_hsv[:,:,1]<60)&(jaw_hsv[:,:,2]>100))[disk]))
            if (cy+y0>=config["min_jaw_red_center_y"] and enclosed>=.8*count and
                    fraction>=config["min_circle_red_fraction"] and pale>=.65):
                accepted.append([float(cx+x0),float(cy+y0),float(radius)])
    result["held_piece_circles"]=accepted
    if not accepted:
        return dict(result,reason="held_round_face_not_confirmed")
    return dict(result,passed=True,reason="source_clear_and_held_red_round_face")
