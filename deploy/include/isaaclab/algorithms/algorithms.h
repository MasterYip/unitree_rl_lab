// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <iostream>
#include <mutex>

namespace isaaclab
{

class Algorithms
{
public:
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;
    virtual void reset() {}

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }

    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        action.resize(output_shape[1]);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // make sure all input names are in obs
        for (const auto& name : input_names) {
            if (obs.find(name) == obs.end()) {
                throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
            }
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(int i(0); i<input_names.size(); ++i)
        {
            const std::string name_str(input_names[i]);
            auto& input_data = obs.at(name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_sizes[i], input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(), output_names.data(), 1);

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), floatarr, output_shape[1] * sizeof(float));
        return action;
    }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};

class OrtRunnerLSTM : public Algorithms
{
public:
    OrtRunnerLSTM(std::string model_path)
    {
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_lstm_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        // Get all input info
        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get all output info (LSTM models have 3 outputs: actions, h_out, c_out)
        for (size_t i = 0; i < session->GetOutputCount(); ++i) {
            Ort::TypeInfo output_type = session->GetOutputTypeInfo(i);
            output_shapes.push_back(output_type.GetTensorTypeAndShapeInfo().GetShape());
            auto output_name = session->GetOutputNameAllocated(i, allocator);
            output_names.push_back(output_name.release());
        }

        // Find hidden size from h_in input
        hidden_size_ = 0;
        for (size_t i = 0; i < input_names.size(); ++i) {
            if (std::string(input_names[i]) == "h_in") {
                hidden_size_ = input_shapes[i][2]; // shape: [1, 1, hidden_size]
                break;
            }
        }
        h_state_ = std::vector<float>(hidden_size_, 0.0f);
        c_state_ = std::vector<float>(hidden_size_, 0.0f);

        action.resize(output_shapes[0][1]); // first output is "actions"
    }

    void reset() override
    {
        std::fill(h_state_.begin(), h_state_.end(), 0.0f);
        std::fill(c_state_.begin(), c_state_.end(), 0.0f);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) override
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // Add hidden states to obs map
        obs["h_in"] = h_state_;
        obs["c_in"] = c_state_;

        // Create input tensors from all input names
        std::vector<Ort::Value> input_tensors;
        for (int i(0); i < input_names.size(); ++i)
        {
            const std::string name_str(input_names[i]);
            if (obs.find(name_str) == obs.end()) {
                throw std::runtime_error("Input name " + name_str + " not found in observations.");
            }
            auto& input_data = obs.at(name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info,
                input_data.data(), input_sizes[i],
                input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model with all outputs
        auto output_tensors = session->Run(Ort::RunOptions{nullptr},
            input_names.data(), input_tensors.data(), input_tensors.size(),
            output_names.data(), output_names.size());

        // Extract action (first output)
        float* actions = output_tensors[0].GetTensorMutableData<float>();
        {
            std::lock_guard<std::mutex> lock(act_mtx_);
            std::memcpy(action.data(), actions, action.size() * sizeof(float));
        }

        // Update hidden states from outputs
        if (output_tensors.size() >= 3) {
            float* h_out = output_tensors[1].GetTensorMutableData<float>();
            float* c_out = output_tensors[2].GetTensorMutableData<float>();
            std::memcpy(h_state_.data(), h_out, hidden_size_ * sizeof(float));
            std::memcpy(c_state_.data(), c_out, hidden_size_ * sizeof(float));
        }

        return action;
    }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<std::vector<int64_t>> output_shapes;

    size_t hidden_size_;
    std::vector<float> h_state_;
    std::vector<float> c_state_;
};
};