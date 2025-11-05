#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace cwcp_sample{
  void sample0();
  class sample0Item : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample0Item>("sample0Item"); }
  protected:
    virtual void main() override{ sample0(); return;}
  };
  typedef cnoid::ref_ptr<sample0Item> sample0ItemPtr;

  class CWCPSamplePlugin : public cnoid::Plugin
  {
  public:
    CWCPSamplePlugin() : Plugin("CWCPSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample0Item::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(cwcp_sample::CWCPSamplePlugin)
