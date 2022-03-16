# Project

## Mission 1: Line Detection by using Perspective Transform

### 1. Set Points of Source Image  
<img src="https://user-images.githubusercontent.com/53277342/158499347-ddb89745-b70b-4c07-a6a4-70e18fef85f9.png" width="40%"/>

``` python
top_row_offset = Height / 5 * 2
below_row_offset = Height / 3
top_col_offset = Width / 5
below_col_offset = Width / 20

src_tl = [top_col_offset, top_row_offset]
src_tr = [Width - top_col_offset, top_row_offset]
src_bl = [0 + below_col_offset, Height - below_row_offset]
src_br = [Width - below_col_offset, Height - below_row_offset]
```
문제점
    - 화면이 바뀜에 따라 원근 변환을 위한 영역을 어떻게 선택해야할지 고민
    - 곡선 주행 시 지정 영역 벗어남 문제
해결법
    - 프레임 이미지 저장하여 차선에 해당하는 영역만을 적절히 선택
    - 곡선 주행이어도 극단적으로 꺾이는 경우는 사실상 불가능

2. Set Points of Destination Image
``` python
dst_pt = np.float32([[0, 0], [Width, 0], [0, Height], [Width, Height]])
```

3. Get Translated Image
<img src="https://user-images.githubusercontent.com/53277342/158499389-8b33ac1d-cd18-4cd6-8f35-dd3c24d05ad5.png" width="40%"/>

```python
pers_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)
dst_img = cv2.warpPerspective(frame, pers_mat, (Width, Height))
```

문제점
    - 영상의 밝기, 배경이 고르지 않은 까닭에 차선 인식이 제대로 이루어지지 않음
