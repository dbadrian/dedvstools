#include "fann.h"

#include <iostream>
#include <ctime>
#include <string>
#include <sstream>

#include "DEdvs/dedvs_auxiliary.h"

void simulated_annealing(struct fann *ann, int epoch);

int main(int argc, char *argv[])
{
    char time_string[200];
    dedvs::getTimeString(time_string);

    std::ostringstream ostr;

    std::cout << "FANN DEDVS CALIBRATION" << std::endl;
    //// Command line arguments
    if(argc < 2 || argc > 4)
    {
        std::cout << "Wrong Usage" << std::endl;
        return 0;
    } 
    else if(argc < 3)
    {
        std::cout << "No testing data provided, 30\% of training data will be used to testing (UNSAFE)" << std::endl;
    }
    else if(argc == 3)
    {
        std::cout << "## Training data: " << argv[1] << std::endl;
        std::cout << "## Testing data: " << argv[2] 
                  << " (Only if ANN improves on this data, the updated ANN will be safed)" << std::endl;
    } 
    else if(argc == 4)
    {
        std::cout << "## Neural Network: " << argv[3] << std::endl;
        std::cout << "## Training data: " << argv[1] << std::endl;
        std::cout << "## Testing data: " << argv[2] 
                  << " (Only if ANN improves on this data, the updated ANN will be safed)" << std::endl;
    }

    std::cout << std::endl;
    
    // Setup
    const float desired_error = (const float) 1.0;
    const unsigned int annealing_per_epochs = 20;
    const unsigned int max_epochs = 500000;
    const unsigned int epochs_between_reports = 1;
    struct fann_train_data *data, *train, *eval;
    struct fann *ann;

    if(argc == 4)
    {
        ann = fann_create_from_file(argv[3]);
    }
    else
    {
        ann = fann_create_standard(4,3,12,12,2);
        fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
        fann_set_activation_function_output(ann, FANN_LINEAR);

        fann_set_training_algorithm(ann,FANN_TRAIN_INCREMENTAL);
        fann_set_learning_rate(ann,0.000001);
    }

    data = fann_read_train_from_file(argv[1]);
    fann_shuffle_train_data(data);

    if(argc==2)
    {
        int nbr_all = fann_length_train_data(data);
        int nbr_train = (int) (nbr_all*0.7);

        train = fann_subset_train_data(data,0,nbr_train);
        eval = fann_subset_train_data(data,nbr_train,nbr_all-nbr_train);
    }
    else
    {
        train = data;
        eval = fann_read_train_from_file(argv[2]);
    }

    std::cout << "# Training Network" << std::endl;

    bool lowered1 = false;
    bool lowered2 = false;
    bool lowered3 = false;
    bool lowered4 = false;    
    float last_error = 99999999.0;
    float last_testdata_error = 99999999.0;

    if(argc==4 && fann_get_learning_rate(ann) <= 0.0065) /* The ANN probably already has seen stages 2 and 3 as well */
    {
        lowered1 = true;
    }
    else if(argc==4 && fann_get_learning_rate(ann) < 0.0025)
    {
        lowered2 = true;
    }
    else if(argc==4 && fann_get_learning_rate(ann) < 0.0015)
    {
        lowered3 = true;
    }
    else if(argc==4 && fann_get_learning_rate(ann) < 0.0009)
    {
        lowered4 = true;
    }

    int epoch = 1;

    double steepness = 0.5;

    do
    {
        fann_reset_MSE(ann);
        fann_train_epoch(ann, train);

        // if(epoch%150 == 0)
        // {
        //     fann_shuffle_train_data(data);
        // }   

        if(epoch%epochs_between_reports == 0)
        {
            std::cout << "Epoch #" << epoch << " | MSE: " << fann_get_MSE(ann);
        }


        if(fann_get_MSE(ann) < last_error)
        {
            if(argc >= 3)
            {
                last_error = fann_get_MSE(ann);
                fann_reset_MSE(ann);
                for(int i = 0; i < fann_length_train_data(eval); ++i)
                {
                    fann_test(ann, eval->input[i], eval->output[i]);
                }

                if(fann_get_MSE(ann) < last_testdata_error)
                {
                    last_testdata_error = fann_get_MSE(ann);
                    fann_save(ann, ("nncalib-" + std::string(argv[1]) + std::string(time_string)+".net").c_str());

                    if(epoch%epochs_between_reports == 0)
                    {
                        std::cout << "\t --- Error on test data decreased.... Updated ANN saved! (MSE: " << fann_get_MSE(ann) << " )" << std::endl;
                    }

                } 
                else
                {
                    if(epoch%epochs_between_reports == 0)
                    {
                        std::cout << "\t --- Error on test data did not decrease (MSE: " << fann_get_MSE(ann) << " )" << std::endl;
                    }
                }
            }
            else
            {
                fann_save(ann, ("nncalib-" + std::string(argv[1]) + std::string(time_string)+".net").c_str());
                last_error = fann_get_MSE(ann);
            }
        } 
        else
        {
            if(epoch%epochs_between_reports == 0)
            {
                std::cout << std::endl;
            }
        }

        if(epoch%50==0)
        {
            fann_shuffle_train_data(data);
        }


        // if(epoch%annealing_per_epochs == 0)// && !lowered2)
        // {
        //     simulated_annealing(ann,epoch);
        //     std::cout << " (Anneal!)" << std::endl;
        // }         


        // if(fann_get_MSE(ann) < 50.0 && fann_get_MSE(ann) > 15.0 && lowered1 == false)
        // {
        //     fann_set_learning_rate(ann,0.0065);
        //     std::cout << "Lowering the learning rate... (1)"  << std::endl;
        //     lowered1 = true;
        // }

        // if(fann_get_MSE(ann) < 15.0 && fann_get_MSE(ann) > 5.0 && lowered2 == false)
        // {
        //     fann_set_learning_rate(ann,0.0025);
        //     std::cout << "Lowering the learning rate... (2)"  << std::endl;
        //     lowered2 = true;
        // }

        // if(fann_get_MSE(ann) < 5.0 && lowered3 == false)
        // {
        //     fann_set_learning_rate(ann,0.0015);
        //     std::cout << "Lowering the learning rate... (3)"  << std::endl;
        //     lowered3 = true;
        // }

        // if(fann_get_MSE(ann) < 3.5 && lowered4 == false)
        // {
        //     fann_set_learning_rate(ann,0.0009);
        //     std::cout << "Lowering the learning rate... (4)"  << std::endl;
        //     lowered4 = true;
        // }

    } while (fann_get_MSE(ann) > desired_error && ++epoch < max_epochs);




    std::cout << "# Testing Network" << std::endl;
    fann_reset_MSE(ann);
    for(int i = 0; i < fann_length_train_data(eval); ++i)
    {
        fann_test(ann, eval->input[i], eval->output[i]);
    }
    
    std::cout << "MSE error on test data:" << fann_get_MSE(ann) << std::endl;

    ostr.str("");
    ostr.clear();
    ostr << fann_get_MSE(ann);
    fann_save(ann, ("nncalib-MSE" + ostr.str() + "-" + std::string(argv[1]) + std::string(time_string)+".net").c_str());

    fann_destroy(ann);

    return 0;
}

void simulated_annealing(struct fann *ann, int epoch)
{
        if(epoch==0) return;

        /* scaling factor to decrease amount of simulated annealing for increasing epochs
                i.e. influence of simulated annealing decreases over time */
        double alpha = exp(-(double)epoch/500.0-0.2);

        /* do not apply weight change, when amount is too small */
        if(alpha<1e-9) return;

        
        fann_type *last_weight;
        fann_type *weights = ann->weights;

        last_weight = weights + ann->total_connections;

        double u_weight;
        double rand_add;
        double r;

        /* for all weights of the networks */
        for(;weights!=last_weight;weights++)
        {
                /* store the current weight */
                u_weight = *weights;
                /* compute random value between -1 and 1 */
                r = 2.0*((double)rand() / (double)RAND_MAX - 0.5);
                /* weight random value with scaling factor such that amount decreases over time */
                rand_add = alpha * r * u_weight/10;
                /* change the weight */
                *weights += rand_add;
        }
}