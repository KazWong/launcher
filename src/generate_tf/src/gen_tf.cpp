#include "generate_tf/gen_tf.h"

using namespace gen_tf;

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif


Gentf::Gentf() {
   
}

Gentf::~Gentf() {
  
}

void Gentf::write_to_yaml(string parent_f, string child_f, double x,double y,double z, double rx, double ry, double rz, bool is_static_tf)
{
    cout << "child_f is: "<<child_f <<endl;
    YAML::Node node;
    YAML::Node node_;
    std::ofstream fout;
    std::ofstream fout_index;
    YAML::Node config;
    std::string static_tf_path="/home/sae/te_lab/src/generate_tf/param/static_tf.yaml";
    std::string static_tf__index_path="/home/sae/te_lab/src/generate_tf/param/static_tf_index.yaml";
    std::string tf_broadcaster_path="/home/sae/te_lab/src/generate_tf/param/tf_broadcastor.yaml";
    std::string tf_broadcaster_index_path="/home/sae/te_lab/src/generate_tf/param/tf_broadcastor_index.yaml";
    node["parent_frame"] = parent_f;
    node["transform"] = YAML::Load("[]");
    node["transform"][0] = x;
    node["transform"][1] = y;
    node["transform"][2] = z;
    node["transform"][3] = rx;
    node["transform"][4] = ry;
    node["transform"][5] = rz;
    node_[child_f] = node; 

    if (is_static_tf) {
        fout.open( static_tf_path,ios::out|ios::app);
        fout_index.open( static_tf__index_path,ios::out|ios::app);
        config = YAML::LoadFile(static_tf__index_path);
    } else {
        fout.open( tf_broadcaster_path,ios::out|ios::app);
        fout_index.open( tf_broadcaster_index_path,ios::out|ios::app);
        config = YAML::LoadFile(tf_broadcaster_index_path);
    }


    fout << node_;
    fout << "\n";

    int size = config.size();
    string index = "index";
    size++;
    string new_index = index+to_string(size);

    YAML::Node new_config;
    new_config[new_index] = child_f;

    fout_index << new_config;
    fout_index << "\n";
}








