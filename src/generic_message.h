/*
This is the header file for the generic message class that accepts templated data types and provides
a standard getTime(), getSize(), getSeq(), methods for all message types.
*/

#pragma once

template <typename T>
class GenericMessage {
public:
    // Constructor
    GenericMessage(const T& message);

    // Destructor
    ~GenericMessage() {

    }

    // Get the time the message was created
    double getTime() {
        //TODO: Implement this
        return 0.0;
    }

    // Get the size of the message
    int getSize() {
        //TODO: Implement this. Should return sensible data size
        return 0;
    }

    // Get the sequence number of the message
    int getSeq() {
        //TODO: Implement this, should return seq num 
        return 0;
    }

private:
    //
};
