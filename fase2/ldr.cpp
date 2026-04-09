#include "ldr.h"
LDR::LDR(){
	// Initial parameters of the log function of the sensor (LDR)
	// LED left
    m = -0.8492; 
	b = 5.22; 
    // LED right
    //m = -0.86;
    //b = 6.1;
	// Initialization of the buffer (for the filter)
	bufferIndex = 0;
	bufferFilled = false;
	for(int i = 0; i< MEDIAN_BUFFER_SIZE; i++){
		adc_buffer[i]= 0;
	}
	last_median_adc = 0;
		
}

// Quicksort implementation
void LDR::swap(int* a, int*b){
	int t = *a;
	*a = *b;
	*b = t;
}

int LDR::partition(int* array, int low, int high){
	int pivot = array[high];
	int i = (low -1);
	
	for (int j = low; j <= high - 1; j++) {
        	if (array[j] < pivot) {
            		i++;
            		swap(&array[i], &array[j]);
        	}
    	}
    	swap(&array[i + 1], &array[high]);
    	return (i + 1);
}

void LDR::quickSort(int* array, int low, int high) {
    if (low < high) {
        int pi = partition(array, low, high);

        quickSort(array, low, pi - 1);
        quickSort(array, pi + 1, high);
    }
}

// Wrapper to initiate QuickSort
void LDR::sortArray(int* array, int size) {
    quickSort(array, 0, size - 1);
}


// ADC Reading using Median Filter with Circular Buffer and Transient Detection
float LDR::readVoltage() {
    // New reading 
    int new_adc = analogRead(LDR_PIN);

    // Transient detection 
    if (bufferFilled && abs(new_adc - last_median_adc) > TRANSIENT_THRESHOLD) {
        // If is detected
        for (int i = 0; i < MEDIAN_BUFFER_SIZE; i++) {
            adc_buffer[i] = new_adc;
        }
    } else {
        // Usual behaviour of the buffer
        adc_buffer[bufferIndex] = new_adc;
        bufferIndex++;

        // Wrap around if we reach the end
        if (bufferIndex >= MEDIAN_BUFFER_SIZE) {
            bufferIndex = 0;
            bufferFilled = true;
        }
    }

    // Determine how much valid data we have
    int currentSize = bufferFilled ? MEDIAN_BUFFER_SIZE : bufferIndex;
    if (currentSize == 0) return 0.0;

    // Copy to a temp array so we can sort without destroying the chronological order
    int tempBuffer[MEDIAN_BUFFER_SIZE];
    for (int i = 0; i < currentSize; i++) {
        tempBuffer[i] = adc_buffer[i];
    }

    // Sort the temp array using QuickSort
    sortArray(tempBuffer, currentSize);

    // Get the median value
    int median_adc;
    if (currentSize % 2 == 0) {
        median_adc = (tempBuffer[currentSize / 2 - 1] + tempBuffer[currentSize / 2]) / 2;
    } else {
        median_adc = tempBuffer[currentSize / 2];
    }

    // Save the median to the next reading
    last_median_adc = median_adc;

    // Convert ADC level to voltage
    return (float(median_adc) * vcc) / (ADC_MAX - 1);
}

// ADC Reading (with averaging)
float LDR::readVoltage_avg(int samples){
	int adc = 0;
	for (int i = 0; i < samples; i++){
		adc += analogRead(LDR_PIN);
	}
	float avg_adc = float(adc) / samples;
	return (avg_adc* vcc)/(ADC_MAX - 1); // Level starts at 0
}

// R_LDR value (voltage to resistance)
float LDR::R_LDR_compute(float v){
	// Physical limit of 1 bit in the ADC
	float min_voltage = vcc/ ADC_MAX;
	// Condition to avoid division by zero (limit to the ADC resolution)
	if(v < min_voltage){
		v = min_voltage;
	}
	
	return (r_read_circuit*(vcc-v))/v;
}

// Computation of the Lux values from R_LDR (resistance to Lux)
float LDR::luxCompute(float r_ldr){
	float logLux = (log10(r_ldr)- b) / m;
	return pow(10, logLux);
}


// Full reading of the illuminance
float LDR::readLux(){
	float v = readVoltage();
	float r_ldr = R_LDR_compute(v);
	return luxCompute(r_ldr);
}

void LDR::calibrate_b(float knownLux){
	for(int i = 0; i < MEDIAN_BUFFER_SIZE; i++) readVoltage();
	//float v = readVoltage_avg();
	float v = readVoltage();
	float r_ldr = R_LDR_compute(v);
	b = log10(r_ldr) - m * log10(knownLux);
       	Serial.print("Calibrated b: ");
	Serial.println(b);	
}


void LDR::calibrate_m(LED& led) {
    Serial.println("Initializing the slope (m) calibration.");
    Serial.println("Warning: The box should be closed");
    
    // Data formating for plot
    Serial.println("Duty,log10_Duty,V_LDR,R_LDR,log10_R");

    float sum_x = 0;
    float sum_y = 0;
    float sum_xy = 0;
    float sum_x2 = 0;
    int n = 0;

    // Duty cycle sweep from 0.1 to 1.0
    for (float duty = 0.1; duty <= 1.0; duty += 0.1) {
        led.setDuty(duty);
        delay(2000);
        
	// Clean older readings from the buffer that were 'paused' during delay
	for(int i = 0; i < MEDIAN_BUFFER_SIZE; i++) readVoltage();

        float v = readVoltage();
        float r = R_LDR_compute(v);
        
        // Linear regression variables (y = m*x + b) 
        float x = log10(duty);
        float y = log10(r);
        
        // Sums in the m formula
        sum_x += x; // sum of x values 
        sum_y += y; // sum of y values
        sum_xy += (x * y); // sum of x*y
        sum_x2 += (x * x); // sum of x^2
        n++;
        
        // Formated data for MATLAB processing
        Serial.print(duty, 2); Serial.print(",");
        Serial.print(x, 4); Serial.print(",");
        Serial.print(v, 4); Serial.print(",");
        Serial.print(r, 2); Serial.print(",");
        Serial.print(y, 4); Serial.println(";");
    }
    
    // Turn off the LED after calibration
    led.off();

    // m computation from mean square
    float numerator = (n * sum_xy) - (sum_x * sum_y);   
    float denominator = (n * sum_x2) - (sum_x * sum_x);
    
    m = numerator / denominator; // computation of m, by linear mean square 	

    // Print of the calibrated final results
    Serial.print("Calibrated (m): ");
    Serial.println(m, 4);
       
}

float LDR::get_m(){
    return m;
}

float LDR::get_b(){
    return b;
}
