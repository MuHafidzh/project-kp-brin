#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <vector>

namespace py = pybind11;

class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *msg) noexcept override
    {
        // Implement your logging mechanism here
        std::cout << msg << std::endl;
    }
};

class TensorRTBert
{
public:
    TensorRTBert(const std::string &engine_file_path)
    {
        // Load TensorRT engine
        std::ifstream file(engine_file_path, std::ios::binary);
        if (!file.good())
        {
            throw std::runtime_error("Failed to open engine file.");
        }
        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> buffer(size);
        file.read(buffer.data(), size);
        file.close();

        runtime = nvinfer1::createInferRuntime(logger);
        engine = runtime->deserializeCudaEngine(buffer.data(), size);
        context = engine->createExecutionContext();
    }

    ~TensorRTBert()
    {
        delete context;
        delete engine;
        delete runtime;
    }

    std::vector<float> infer(py::array_t<int32_t> input_ids, py::array_t<int32_t> attention_mask)
    {
        // Allocate GPU memory
        int32_t *d_input_ids;
        int32_t *d_attention_mask;
        float *d_output;
        cudaMalloc(reinterpret_cast<void **>(&d_input_ids), input_ids.size() * sizeof(int32_t));
        cudaMalloc(reinterpret_cast<void **>(&d_attention_mask), attention_mask.size() * sizeof(int32_t));
        cudaMalloc(reinterpret_cast<void **>(&d_output), 7 * sizeof(float)); // Assuming 7 class probabilities

        // Copy data to GPU
        cudaMemcpy(d_input_ids, input_ids.data(), input_ids.size() * sizeof(int32_t), cudaMemcpyHostToDevice);
        cudaMemcpy(d_attention_mask, attention_mask.data(), attention_mask.size() * sizeof(int32_t), cudaMemcpyHostToDevice);

        // Run inference
        void *bindings[] = {d_input_ids, d_attention_mask, d_output};
        context->executeV2(bindings);

        // Retrieve output
        std::vector<float> output(7);
        cudaMemcpy(output.data(), d_output, 7 * sizeof(float), cudaMemcpyDeviceToHost);

        // Free GPU memory
        cudaFree(d_input_ids);
        cudaFree(d_attention_mask);
        cudaFree(d_output);

        return output;
    }

private:
    nvinfer1::IRuntime *runtime;
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    Logger logger;
};

PYBIND11_MODULE(tensorrtbert, m)
{
    py::class_<TensorRTBert>(m, "TensorRTBert")
        .def(py::init<const std::string &>())
        .def("infer", &TensorRTBert::infer);
}