import tensorflow
tensorflow.keras.backend.clear_session()
detect_fn = tensorflow.saved_model.load('/home/roman/models/bricks_model/saved_model_trt')
print('Loaded')
