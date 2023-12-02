# FIR-Filter

This repository contains a lightweight implementation of a Finite Impulse Response (FIR) filter in C, designed for embedded systems and signal processing applications. The FIR filter is a digital filter that efficiently processes input signals, providing a smooth and controlled output based on a predefined impulse response.

**Features:**
* Modular Structure: The implementation follows a modular structure, with a header file (FIR_Filter.h) defining the filter interface and a corresponding source file (FIR_Filter.c) containing the filter logic.
* Configurable Length: The filter length is configurable through the FIR_FILTER_LENGTH constant, allowing users to adapt the filter to different signal processing requirements.
* Efficient Initialization: The FIRFilter_Init function efficiently initializes the filter, ensuring proper setup before processing input data.
* Update Functionality: The FIRFilter_Update function updates the filter with new input, applying the FIR filter algorithm to generate the filtered output.

**Usage:**
* Include the FIR_Filter.h header file in your project.
* Initialize the FIR filter using FIRFilter_Init.
* Call FIRFilter_Update with new input data to obtain the filtered output.
```C
// Create an instance of the FIR filter
FIRFilter Filter;

// Initialize the filter
FIRFilter_Init(&Filter);

// Update the filter with new input and obtain the filtered output
float filteredOutput = FIRFilter_Update(&Filter, input);
```

**Impulse Response:**
* The FIR filter uses a predefined impulse response defined in the source file (FIR_Filter.c). Users can modify the FIR_IMPULSE_RESPONSE array to customize the filter characteristics according to their specific application requirements.
