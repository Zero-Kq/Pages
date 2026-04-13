+++
title = 'K230 部署 YOLOv8'
description = "在 K230 开发板上部署 YOLOv8 目标检测模型"
date = '2026-04-13'
draft = true
tags = ["嵌入式", "python", "视觉", "学习笔记"]
categories = ["感知"]
toc = true
math = false
+++

## 介绍

部署 YOLOv8 目标检测算法到 K230 开发板上运行，本文记录完整的部署流程。

---

## 环境准备

### 软件依赖

- CanMV IDE
- 已训练好的 YOLOv8 模型（best.pt）
- Python 环境（建议使用 conda）

### 硬件要求

- K230 开发板（已刷好固件）
- 摄像头

---

## 模型转换

### PyTorch → ONNX

将训练好的模型导出为 ONNX 格式，支持模型量化优化。

```python
from ultralytics import YOLO

if __name__ == "__main__":
    # 1. 加载训练好的 YOLO 模型
    model = YOLO(r"D:\CUB_200_2011\bird_5\runs\detect\runs\train\bird_train\weights\best.pt")

    # 2. 调用 YOLO 模型实例的 export 方法
    # 注意：这里不需要重新赋值 model，export 方法会直接生成 ONNX 文件
    model.export(
        format="onnx",  # 指定导出格式为 ONNX
        simplify=True,   # 简化 ONNX 模型
        imgsz=640,      # 输入图像尺寸
        opset=12        # ONNX 算子集版本
    )

    print("ONNX 模型导出成功！文件会保存在 best.pt 同目录下")
```

### ONNX → KModel

在 Windows 环境下完成模型转换。

#### 1. 安装 .NET 7.0

下载并安装 [.NET 7.0](https://dotnet.microsoft.com/zh-cn/download/dotnet/7.0)。

#### 2. 安装 Python 依赖

```bash
pip install nncase onnx onnxruntime onnxsim
```

#### 3. 安装 nncase_kpu

从 [nncase releases](https://github.com/kendryte/nncase/releases) 下载 `nncase_kpu-2*-py2.py3-none-win_amd64.whl`，然后安装：

```bash
pip install nncase_kpu-2*-py2.py3-none-win_amd64.whl
```

#### 4. 验证版本

```bash
pip list | findstr nncase
```

确保 nncase 和 nncase_kpu 版本一致。

#### 5. 复制依赖库

下载 [libomp140.x86_64.zip](https://github.com/kendryte/nncase/files/7594453/libomp140.x86_64.zip)，将 `libomp140.x86_64.dll` 复制到 `C:\Windows\System32`。

#### 6. 下载官方转换工具

```bash
wget https://kendryte-download.canaan-creative.com/developer/k230/yolo_files/test_yolov8.zip
unzip test_yolov8.zip
```

#### 7. 转换为 KModel

```bash
python to_kmodel.py --target k230 --model ../best.onnx --dataset ../test --input_width 640 --input_height 640 --ptq_option 0
```

| 参数 | 说明 |
|------|------|
| model | ONNX 模型路径 |
| dataset | 验证图片文件夹 |
| input_width/height | 输入图像尺寸 |
| ptq_option | 量化选项（0 = int8） |

#### 警告说明

以下警告不影响转换结果：

```terminal
warn: Nncase.Hosting.PluginLoader[0]
      NNCASE_PLUGIN_PATH is not set.
to_kmodel.py:25: DeprecationWarning: `mapping.TENSOR_TYPE_TO_NP_TYPE` is now deprecated and will be removed in a future release.To silence this warning, please use `helper.tensor_dtype_to_np_dtype` instead.
  input_dict['dtype'] = onnx.mapping.TENSOR_TYPE_TO_NP_TYPE[onnx_type.elem_type]
WARNING: The argument `input_shapes` is deprecated. Please use
`overwrite_input_shapes` and/or `test_input_shapes` instead. An error
will be raised in the future.
```

---

## 部署代码

将 KModel 文件放入 K230 开发板，运行以下代码：

```python
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import random
import gc
import sys
import aidemo

# 自定义YOLOv8检测类
class ObjectDetectionApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,max_boxes_num,confidence_threshold=0.5,nms_threshold=0.2,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.labels=labels
        # 模型输入分辨率
        self.model_input_size=model_input_size
        # 阈值设置
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.max_boxes_num=max_boxes_num
        # sensor给到AI的图像分辨率
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 显示分辨率
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        # 检测框预置颜色值
        self.color_four=[(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
                         (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
                         (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
                         (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
                         (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157)]
        # 宽高缩放比例
        self.x_factor = float(self.rgb888p_size[0])/self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1])/self.model_input_size[1]
        # Ai2d实例，用于实现模型预处理
        self.ai2d=Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    # 配置预处理操作，这里使用了resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理
    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            result=results[0]
            result = result.reshape((result.shape[0] * result.shape[1], result.shape[2]))
            output_data = result.transpose()
            boxes_ori = output_data[:,0:4]
            scores_ori = output_data[:,4:]
            confs_ori = np.max(scores_ori,axis=-1)
            inds_ori = np.argmax(scores_ori,axis=-1)
            boxes,scores,inds = [],[],[]
            for i in range(len(boxes_ori)):
                if confs_ori[i] > self.confidence_threshold:
                    scores.append(confs_ori[i])
                    inds.append(inds_ori[i])
                    x = boxes_ori[i,0]
                    y = boxes_ori[i,1]
                    w = boxes_ori[i,2]
                    h = boxes_ori[i,3]
                    left = int((x - 0.5 * w) * self.x_factor)
                    top = int((y - 0.5 * h) * self.y_factor)
                    right = int((x + 0.5 * w) * self.x_factor)
                    bottom = int((y + 0.5 * h) * self.y_factor)
                    boxes.append([left,top,right,bottom])
            if len(boxes)==0:
                return []
            boxes = np.array(boxes)
            scores = np.array(scores)
            inds = np.array(inds)
            # NMS过程
            keep = self.nms(boxes,scores,nms_threshold)
            dets = np.concatenate((boxes, scores.reshape((len(boxes),1)), inds.reshape((len(boxes),1))), axis=1)
            dets_out = []
            for keep_i in keep:
                dets_out.append(dets[keep_i])
            dets_out = np.array(dets_out)
            dets_out = dets_out[:self.max_boxes_num, :]
            return dets_out

    # 绘制结果
    def draw_result(self,pl,dets):
        with ScopedTiming("display_draw",self.debug_mode >0):
            if dets:
                pl.osd_img.clear()
                for det in dets:
                    x1, y1, x2, y2 = map(lambda x: int(round(x, 0)), det[:4])
                    x= x1*self.display_size[0] // self.rgb888p_size[0]
                    y= y1*self.display_size[1] // self.rgb888p_size[1]
                    w = (x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = (y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    pl.osd_img.draw_rectangle(x,y, w, h, color=self.get_color(int(det[5])),thickness=4)
                    pl.osd_img.draw_string_advanced( x , y-50,32," " + self.labels[int(det[5])] + " " + str(round(det[4],2)) , color=self.get_color(int(det[5])))
            else:
                pl.osd_img.clear()

    # 多目标检测 非最大值抑制方法实现
    def nms(self,boxes,scores,thresh):
        """Pure Python NMS baseline."""
        x1,y1,x2,y2 = boxes[:, 0],boxes[:, 1],boxes[:, 2],boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = np.argsort(scores,axis = 0)[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            new_x1,new_y1,new_x2,new_y2,new_areas = [],[],[],[],[]
            for order_i in order:
                new_x1.append(x1[order_i])
                new_x2.append(x2[order_i])
                new_y1.append(y1[order_i])
                new_y2.append(y2[order_i])
                new_areas.append(areas[order_i])
            new_x1 = np.array(new_x1)
            new_x2 = np.array(new_x2)
            new_y1 = np.array(new_y1)
            new_y2 = np.array(new_y2)
            xx1 = np.maximum(x1[i], new_x1)
            yy1 = np.maximum(y1[i], new_y1)
            xx2 = np.minimum(x2[i], new_x2)
            yy2 = np.minimum(y2[i], new_y2)
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            new_areas = np.array(new_areas)
            ovr = inter / (areas[i] + new_areas - inter)
            new_order = []
            for ovr_i,ind in enumerate(ovr):
                if ind < thresh:
                    new_order.append(order[ovr_i])
            order = np.array(new_order,dtype=np.uint8)
        return keep

    # 根据当前类别索引获取框的颜色
    def get_color(self, x):
        idx=x%len(self.color_four)
        return self.color_four[idx]


if __name__=="__main__":
    # 显示模式，默认"hdmi",可以选择"hdmi"和"lcd"
    display_mode="hdmi"
    if display_mode=="hdmi":
        display_size=[1920,1080]
    else:
        display_size=[800,480]
    # 模型路径
    kmodel_path="/sdcard/examples/kmodel/best.kmodel"
    labels = ["Black_footed_Albatross", "Laysan_Albatross", "Sooty_Albatross", "Groove_billed_Ani", "Crested_Auklet"]
    # 其它参数设置
    confidence_threshold = 0.2
    nms_threshold = 0.2
    max_boxes_num = 50
    rgb888p_size=[320,320]

    # 初始化PipeLine
    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    pl.create()
    # 初始化自定义目标检测实例
    ob_det=ObjectDetectionApp(kmodel_path,labels=labels,model_input_size=[640,640],max_boxes_num=max_boxes_num,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,rgb888p_size=rgb888p_size,display_size=display_size,debug_mode=0)
    ob_det.config_preprocess()
    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total",1):
                # 获取当前帧数据
                img=pl.get_frame()
                # 推理当前帧
                res=ob_det.run(img)
                # 绘制结果到PipeLine的osd图像
                ob_det.draw_result(pl,res)
                # 显示当前的绘制结果
                pl.show_image()
                gc.collect()
    except Exception as e:
        sys.print_exception(e)
    finally:
        ob_det.deinit()
        pl.destroy()
```

---

## 参考

- [Kendryte K230 YOLO 大作战](https://www.kendryte.com/k230_canmv/zh/main/zh/example/ai/YOLO%E5%A4%A7%E4%BD%9C%E6%88%98.html)

---