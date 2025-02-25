import numpy as np
from transformers import DistilBertTokenizer
import tensorrtbert
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
engine_path = os.path.join(script_dir, "db69.engine")
engine_db3_path = os.path.join(script_dir, "db3.engine")

# Load the tokenizer
model_name = "distilbert-base-uncased"
tokenizer = DistilBertTokenizer.from_pretrained(model_name)

# Define class labels
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
def init():
    engine = tensorrtbert.TensorRTBert(engine_db3_path)
    return engine

# Function to perform inference
def infer_text(text, engine):
    # Tokenize the input text
    inputs = tokenizer(text, padding="max_length", truncation=True, max_length=512, return_tensors="np")
    input_ids = inputs["input_ids"].astype(np.int32)
    attention_mask = inputs["attention_mask"].astype(np.int32)

    # Run inference
    output = engine.infer(input_ids, attention_mask)

    # Get the label with highest probability
    predicted_label = id2label[np.argmax(output)]

    return predicted_label


if __name__=='__main__':
    # Example usage
    model = init()
    input_text = ["find a someone who can help me",'bottle','laptop']
    for t in input_text:
        predicted_label = infer_text(t, model)
        print(f"Input: {t}")
        print(f"Predicted Label: {predicted_label}")
