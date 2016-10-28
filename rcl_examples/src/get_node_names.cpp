#include <rcl/rcl.h>
#include <iostream>
#include <cstring>
/*
 * Expected usage:
 *
 *    rcl_node_t node = rcl_get_zero_initialized_node();
 *    rcl_node_options_t * node_ops = rcl_node_get_default_options();
 *    rcl_ret_t ret = rcl_node_init(&node, "node_name", node_ops);
 *    // ... error handling and then use the node, but finally deinitialize it:
 *    ret = rcl_node_fini(&node);
 *    // ... error handling for rcl_node_fini()
*/
int main(int argc, char ** argv)
{
    rcl_ret_t ret1 = rcl_init(argc, argv, rcl_get_default_allocator());
//    std::cout << "ret1: " << ret1 << std::endl;

    rcl_node_t node_ptr;
    node_ptr = rcl_get_zero_initialized_node();
    const char * name = "ros_test_node_victor";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    rcl_ret_t ret = rcl_node_init(&node_ptr, name, &node_options);
    
    rcl_strings_t node_names;

    node_names = rcl_get_zero_initialized_strings();

    rcl_ret_t ret3 = rcl_get_node_names(&node_names);
    //std::cout << "ret: " << ret3 << std::endl;

    for(unsigned int i = 0; i < node_names.count; i++){
        std::cout << "->/" << node_names.data[i] << std::endl;
    }

}


