#include <designators/Designator.h>
#include <SWI-cpp.h>
#include <iostream>
#include <string>

designator_integration::Designator *req_desig;

PREDICATE(hello, 1)
{
  std::cout << "Hello baby" << (char *)A1 << std::endl;
  return TRUE;
}

PREDICATE(init_desig, 1)
{
  if(!req_desig)
  {
    req_desig = new designator_integration::Designator();
    req_desig->setValue("type","DETECT");
    designator_integration::KeyValuePair *links = new designator_integration::KeyValuePair("location");
    links->addChild("location","table");
    req_desig->addChild(links);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}
PREDICATE(delete_desig,1)
{
  if(req_desig)
  {
    delete req_desig;
    return TRUE;
  }
  else
    return FALSE;
}

PREDICATE(add_kvp, 2)
{
  std::string key= (std::string)A1;
  std::string value = (std::string)A2;
  designator_integration::KeyValuePair *kvp=new designator_integration::KeyValuePair(key,value);

  if(req_desig)
  {
    req_desig->addChild(kvp);
    delete kvp;
    return TRUE;
  }
}

PREDICATE(print_desig, 1)
{
  if(req_desig !=NULL)
  {
    req_desig->printDesignator();
    return TRUE;
  }
  else
    return FALSE;
}
