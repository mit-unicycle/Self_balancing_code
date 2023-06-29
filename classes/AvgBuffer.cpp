class AvgBuffer {
private:
  int front;
  int rear;
  int count;
  const int MAX_SIZE;
  double * buffer;

public:
    AvgBuffer(int buffSize) : MAX_SIZE(buffSize){
        buffer = new double[MAX_SIZE];
        front = 0;
        rear = -1;
        count = 0;
    }

    bool isEmpty() {
        return count == 0;
    }

    bool isFull() {
        return count == MAX_SIZE;
    }

    void add(double value) {
        if (isFull()) {
        // Buffer is full, discard or handle the overflow
        return;
        }

        rear = (rear + 1) % MAX_SIZE;
        buffer[rear] = value;
        count++;
    }

    double calculate() {
        if (isEmpty()) {
            // Buffer is empty, handle the underflow, Return a default value 
            return 0.0;
        }

        // Calculate the sum of remaining elements in the buffer
        double sum = 0.0;
        int numElements = count;

        for (int i = 0; i < numElements; i++) {
            sum += buffer[(front + i) % MAX_SIZE];
        }

        // Remove the values from the buffer only if it is full
        if (isFull()) {
            front = (front + 1) % MAX_SIZE;
            count--;
        }

        // Calculate and return the average
        double average = sum / numElements;

        return average;
    }
};
