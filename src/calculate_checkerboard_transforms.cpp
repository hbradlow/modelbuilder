#include "model_building.cpp"
using namespace std;

int main(int argc, char*argv[]){

    //keep track of all the frames
    vector<Eigen::Matrix4f> transforms(argc-1); //the transform of the checkerboard for the frame
    vector<string> filenames(argc-1); //the transform of the checkerboard for the frame
    vector<int> success_mask(argc-1); //indicates if a checkerboard was located in the frame
    bool error = 0;

    if(argc>1){
        // Reorient each cloud to line the checkerboards up and concatenate them all into a "master" cloud

        //collect some stats to tell the user how successful the checkerboard location was
        int found_boards = 0;
        int total_clouds = 0;

        cout << "Reorienting the clouds to match their checkerboards..." << endl;
        //load all of the clouds into the "clouds" vector
        #pragma omp parallel for
        for(int i=1; i<argc; i++)
        {
            int index = i-1;
            //load the cloud from the file
            pcl::PointCloud<pcl::PointNormal>::Ptr current (new pcl::PointCloud<pcl::PointNormal>); //buffer to hold the cloud for the current frame
            ColorCloudPtr tmp (new ColorCloud); //a buffer to store the cloud that is loaded from disk
            filenames[index] = argv[i];

            if(loadCloud(argv[i],current,tmp) == -1)
                error = 1;
            
            current.reset();

            if(!error){
                #pragma omp critical
                {
                    total_clouds ++;
                    //update the user of the progress
                    cout << "PERCENT COMPLETE:" << (float)total_clouds/(argc-1) * 100 << endl;
                }

                //calculate the transform of the board
                Eigen::Matrix4f transform;
                int found = getChessBoardPose(tmp,4,9,.02,transform);
                //int found = getChessBoardPose(tmp,6,9,.0172,transform);
                transforms[index] = transform;
                if(found){
                    success_mask[index] = 1; //indicate that the checkerboard was found
                    found_boards ++;
                }
                else
                {
                    success_mask[index] = 0; //indicate that the checkerboard was not found
                }
            }
        }
        writeTransforms("transforms.txt",&transforms,&filenames,&success_mask);
    }
    return error;
}
