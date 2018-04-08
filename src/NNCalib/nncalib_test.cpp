#include "fann.h"

#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "FANN Training Data Converter" << std::endl;
    //// Command line arguments
    if(argc != 3)
    {
        std::cout << "Wrong Usage" << std::endl;
        return 0;
    }

    std::cout << "## Neural Network: " << argv[1] << std::endl;
    std::cout << "## Test Data: " << argv[2] << std::endl;

    struct fann *ann = fann_create_from_file(argv[1]);
    struct fann_train_data *eval = fann_read_train_from_file(argv[2]);

    std::cout << "# Testing Network" << std::endl;
    fann_reset_MSE(ann);
    for(int i = 0; i < fann_length_train_data(eval); ++i)
    {
        fann_test(ann, eval->input[i], eval->output[i]);
    }

    std::cout << "MSE error on test data:" << fann_get_MSE(ann) << std::endl;

    fann_destroy(ann);

    return 0;
}