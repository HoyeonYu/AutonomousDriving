# Project

## Mission 1: Line Detection by using Perspective Transform

1. Set Points of Source Image  
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
