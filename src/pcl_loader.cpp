#include "vofod/pc_loader.h"

#include <sstream>
#include <algorithm>
#include <iterator>

/* // modified from http://www.martinbroadhurst.com/how-to-split-a-string-in-c.html */
/* std::vector<std::string> split(const std::string& str, char delim = ' ') */
/* { */
/*   std::vector<std::string> ret; */
/*   std::stringstream ss(str); */
/*   std::string token; */
/*   while (std::getline(ss, token, delim)) */
/*     ret.push_back(token); */
/*   return ret; */
/* } */

pc_t::Ptr load_cloud(const std::string& filename)
{
  pc_t::Ptr cloud = boost::make_shared<pc_t>();
  std::ifstream fs;
  fs.open(filename.c_str(), std::ios::binary);
  if (!fs.is_open() || fs.fail())
  {
    PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno)); 
    fs.close();
    return nullptr;
  }

  const std::string ftype = filename.substr(filename.find_last_of(".") + 1);

  std::string line;
  size_t n_pts = 0;

  // check if the type has the number of points on the first line and if so, extract it
  if (ftype == "pts")
  {
    std::getline(fs, line);
    boost::trim(line);
    n_pts = float(atof(line.c_str()));
  }
  // otherwise count the number of endlines in the file
  else
  {
    n_pts = std::count(std::istreambuf_iterator<char>(fs), std::istreambuf_iterator<char>(), '\n');
    // recover state
    fs.clear() ; // previous reading may set eofbit
    fs.seekg(0);
  }
  cloud->reserve(n_pts);
  
  while (!fs.eof())
  {
    std::getline(fs, line);
    // Ignore empty lines
    if (line.empty())
      continue;

    // Tokenize the line
    boost::trim(line);
    /* const std::vector<std::string> st = split(line, ' '); */
    std::vector<std::string> st;
    boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

    if (st.size() < 3)
    {
      PCL_WARN("Read line with a wrong number of elements: %d (expected at least 3). The line: '%s'.", (int)st.size(), line.c_str()); 
      continue;
    }

    cloud->push_back(pt_t(
          float(atof(st[0].c_str())),
          float(atof(st[1].c_str())),
          float(atof(st[2].c_str()))
          ));

    if (cloud->size() % 100000 == 0)
    {
      if (!cloud->empty())
        std::cout << "\r";
      PCL_INFO("Loaded %lu/%lu points so far.", cloud->size(), n_pts);
    }
  }
  std::cout << "\n";
  fs.close();

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}
