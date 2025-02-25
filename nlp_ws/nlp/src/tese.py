import tensorrt as trt
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
from transformers import DistilBertTokenizer

# Load the tokenizer
model_name = "distilbert-base-uncased"
tokenizer = DistilBertTokenizer.from_pretrained(model_name)

# Define class labels (modify this according to your model)
id2label = {
    0: "Laptop",
    1: "Dispenser",
    2: "Book",
    3: "Glass",
    4: "Chair",
    5: "Person",
    6: "Bottle"
}

# Load TensorRT engine
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
def load_engine(engine_file_path):
    with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

engine = load_engine("db3.engine")
context = engine.create_execution_context()

# Allocate memory on GPU
def allocate_buffers():
    input_ids = np.zeros((1, 512), dtype=np.int32)
    attention_mask = np.zeros((1, 512), dtype=np.int32)
    output = np.empty(7, dtype=np.float32)  # 7 class probabilities

    d_input_ids = cuda.mem_alloc(input_ids.nbytes)
    d_attention_mask = cuda.mem_alloc(attention_mask.nbytes)
    d_output = cuda.mem_alloc(output.nbytes)

    return d_input_ids, d_attention_mask, d_output, output

d_input_ids, d_attention_mask, d_output, output = allocate_buffers()

# Function to perform inference
def infer_text(text):
    # Tokenize the input text
    inputs = tokenizer(text, padding="max_length", truncation=True, max_length=512, return_tensors="np")
    input_ids = inputs["input_ids"].astype(np.int32)
    attention_mask = inputs["attention_mask"].astype(np.int32)

    # Copy data to GPU
    cuda.memcpy_htod(d_input_ids, input_ids)
    cuda.memcpy_htod(d_attention_mask, attention_mask)

    # Run inference
    context.execute_v2([int(d_input_ids), int(d_attention_mask), int(d_output)])

    # Retrieve output
    cuda.memcpy_dtoh(output, d_output)

    # Get the label with highest probability
    predicted_label = id2label[np.argmax(output)]
    return predicted_label

# Example usage
input_text = "bottle"
predicted_label = infer_text(input_text)
print(f"Input: {input_text}")
print(f"Predicted Label: {predicted_label}")
