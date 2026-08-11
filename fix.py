import onnx
from onnx import helper

# 1. Load your existing ONNX model (the one you exported on Windows)
model_path = "best.onnx"
model = onnx.load(model_path)
graph = model.graph

# 2. Get the current output node
original_output = graph.output[0]
output_name = original_output.name

# 3. Rename the old output so we can intercept it
intercept_name = output_name + "_transposed"
for node in graph.node:
    for i, out in enumerate(node.output):
        if out == output_name:
            node.output[i] = intercept_name

# 4. Create a Transpose node to flip dimensions 1 and 2
transpose_node = helper.make_node(
    'Transpose',
    inputs=[intercept_name],
    outputs=[output_name],
    name='Fix_Isaac_ROS_Transpose',
    perm=[0, 2, 1]  # Flips (1, 5, 19320) -> (1, 19320, 5)
)

# 5. Append the new node to the graph
graph.node.append(transpose_node)

# 6. Update the output shape metadata so TensorRT knows the new size
dim_1 = original_output.type.tensor_type.shape.dim[1].dim_value
dim_2 = original_output.type.tensor_type.shape.dim[2].dim_value
original_output.type.tensor_type.shape.dim[1].dim_value = dim_2
original_output.type.tensor_type.shape.dim[2].dim_value = dim_1

# 7. Save the fixed model over the old one
onnx.save(model, model_path)
print(f"Successfully transposed ONNX model! New shape: (1, {dim_2}, {dim_1})")
