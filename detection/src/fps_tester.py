import cv2
import numpy as np
import time
import tensorflow

if __name__ == '__main__':
    input_saved_model_dir='/home/roman/models/bricks_model/saved_model_trt'

    img = cv2.imread("/mnt/desktop/snapshots/near.jpg")
    print(img.shape)
    im_height, im_width, _ = img.shape
    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    input_tensor = np.expand_dims(img, 0)

    print(" Loading model...")
    tensorflow.keras.backend.clear_session()
    tic = time.perf_counter()
    detect_fn = tensorflow.saved_model.load(input_saved_model_dir)
    toc = time.perf_counter()
    print(f" ...model loaded in {toc-tic:0.2f} seconds")

    BLOCK_SIZE=5
    while True:
        tic = time.perf_counter()
        for i in range(0, (int)(BLOCK_SIZE)):
            det = detect_fn(input_tensor)
            print(f"Found {len(det['detection_boxes'][0].numpy())} boxes")
        toc = time.perf_counter()
        fps = BLOCK_SIZE/(toc-tic)
        print(f"FPS {fps:0.2f}")

