import tensorflow as tf
import numpy as np
from tensorflow.python.compiler.tensorrt import trt_convert as trt
#tf.compat.v1.disable_eager_execution()

input_saved_model_dir='/home/roman/models/bricks_model/saved_model'
output_saved_model_dir='/home/roman/models/bricks_model/saved_model_trt'

from tensorflow.python.compiler.tensorrt import trt_convert as trt

import tensorflow
conversion_params = trt.DEFAULT_TRT_CONVERSION_PARAMS
conversion_params = conversion_params._replace(
    max_workspace_size_bytes=(1<<32))
conversion_params = conversion_params._replace(precision_mode="FP16")
conversion_params = conversion_params._replace(
    maximum_cached_engines=100)


# converter=tensorflow.experimental.tensorrt.Converter(
#     input_saved_model_dir=input_saved_model_dir,
#     conversion_params=conversion_params
# )

converter = trt.TrtGraphConverterV2(
    input_saved_model_dir=input_saved_model_dir,
    conversion_params=conversion_params)
converter.convert()
def my_input_fn():
    inp1 = np.random.normal(size=(1,1, 600, 800, 3),scale=255).astype(np.uint8)
    yield (inp1)

converter.build(input_fn=my_input_fn)
converter.save(output_saved_model_dir)

