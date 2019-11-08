#include "gtest/gtest.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <thread>
#include <vector>

// upstream bug in IPositionDirect.h, remove in YARP 3.2+
#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR < 2
# include <yarp/os/Vocab.h>
#endif

#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include "FutureTask.hpp"
#include "DeviceMapper.hpp"

namespace
{
    struct DummyPositionDirectRawImpl : public yarp::dev::IPositionDirectRaw
    {
        virtual bool getAxes(int * axes) override
        { *axes = 1; return true; }

        virtual bool setPositionRaw(int j, double ref) override
        { return ref >= 0.0 ? true : false; }

        virtual bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override
        { return false; }

        virtual bool setPositionsRaw(const double * refs) override
        { return false; }

        virtual bool getRefPositionRaw(int joint, double * ref) override
        { *ref = joint; return true; }

        virtual bool getRefPositionsRaw(double * refs) override
        { int axes; bool ret; return ret = getAxes(&axes), std::iota(refs, refs + axes, 0.0), ret; };

        virtual bool getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
        { std::copy_n(joints, n_joint, refs); return true; }
    };

    template<unsigned int N>
    struct JointDriver : public yarp::dev::DeviceDriver,
                         public DummyPositionDirectRawImpl
    {
        virtual bool getAxes(int * axes) override
        { *axes = N; return true; }

        static std::string name()
        { return "JointDriver" + std::to_string(N); }

        static yarp::dev::DriverCreator * makeCreator()
        { return new yarp::dev::DriverCreatorOf<JointDriver<N>>(name().c_str(), "", name().c_str()); }
    };
}

namespace roboticslab
{

/**
 * @ingroup yarp_devices_tests
 * @brief ...
 */
class DeviceMapperTest : public testing::Test
{
public:
    DeviceMapperTest() : dummy(nullptr)
    { }

    virtual void SetUp()
    { }

    virtual void TearDown()
    {
        delete dummy;

        for (int i = 0; i < drivers.size(); i++)
        {
            delete drivers[i]->poly;
        }
    }

protected:
    yarp::dev::IPositionDirectRaw * getDummy()
    {
        if (!dummy)
        {
            dummy = new DummyPositionDirectRawImpl;
        }

        return dummy;
    }

    template<typename T>
    yarp::dev::PolyDriver * getDriver()
    {
        yarp::dev::Drivers::factory().add(T::makeCreator());

        yarp::os::Property config;
        config.put("device", T::name());

        auto * driver = new yarp::dev::PolyDriver(config);
        drivers.push(driver, T::name().c_str());
        return driver;
    }

    static constexpr double EPSILON = 1e-9;

private:
    yarp::dev::IPositionDirectRaw * dummy;
    yarp::dev::PolyDriverList drivers;
};

TEST_F(DeviceMapperTest, SequentialTask)
{
    const int joint = 4;

    auto taskFactory = std::unique_ptr<FutureTaskFactory>(new SequentialTaskFactory);
    auto task = taskFactory->createTask();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, 1);
    ASSERT_EQ(task->size(), 1);
    ASSERT_TRUE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, -1);
    ASSERT_EQ(task->size(), 1);
    ASSERT_FALSE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, 1);
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, -1);
    ASSERT_EQ(task->size(), 2);
    ASSERT_FALSE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    double ref = 0.0;
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref);

    ASSERT_EQ(task->size(), 1);
    ASSERT_NEAR(ref, 0.0, EPSILON); // not dispatched yet
    ASSERT_TRUE(task->dispatch());
    ASSERT_NEAR(ref, joint, EPSILON);
}

TEST_F(DeviceMapperTest, ParallelTask)
{
    const int joint = 4;

    auto taskFactory = std::unique_ptr<FutureTaskFactory>(new ParallelTaskFactory(2));
    auto task = taskFactory->createTask();
    ASSERT_EQ(task->size(), 0);

    double ref1 = 0.0;
    double ref2 = 0.0;
    double ref3 = 0.0;
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref1);
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref2);
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref3);

    ASSERT_EQ(task->size(), 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_NEAR(ref1, joint, EPSILON);
    ASSERT_NEAR(ref2, joint, EPSILON);
    ASSERT_NEAR(ref3, 0.0, EPSILON); // not dispatched yet
    ASSERT_TRUE(task->dispatch());
    ASSERT_NEAR(ref3, joint, EPSILON);
}

TEST_F(DeviceMapperTest, RawDevice)
{
    RawDevice rd(getDriver<JointDriver<1>>());
    auto * p = rd.getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p, nullptr);
}

TEST_F(DeviceMapperTest, DeviceMapper)
{
    DeviceMapper mapper;

    // DeviceMapper::registerDevice

    ASSERT_TRUE(mapper.registerDevice(getDriver<JointDriver<1>>())); // [0]
    ASSERT_TRUE(mapper.registerDevice(getDriver<JointDriver<2>>())); // [1-2]
    ASSERT_TRUE(mapper.registerDevice(getDriver<JointDriver<3>>())); // [3-5]
    ASSERT_TRUE(mapper.registerDevice(getDriver<JointDriver<4>>())); // [6-9]

    // DeviceMapper::getDevice

    int axes1, axes2, axes3, axes4;
    double ref0, ref1, ref2, ref3;

    const int localIndex0 = 0;
    const int localIndex1 = 1;
    const int localIndex2 = 2;
    const int localIndex3 = 3;

    // device 1 [0]

    auto devAtIndex0 = mapper.getDevice(0);
    auto * p0 = std::get<0>(devAtIndex0)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p0, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex0), localIndex0);

    ASSERT_TRUE(p0->getAxes(&axes1));
    ASSERT_EQ(axes1, 1);

    ASSERT_TRUE(p0->getRefPositionRaw(localIndex0, &ref0));
    ASSERT_NEAR(ref0, localIndex0, EPSILON);

    // device 2 [1-2]

    auto devAtIndex1 = mapper.getDevice(1);
    auto * p1 = std::get<0>(devAtIndex1)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p1, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex1), localIndex0);

    auto devAtIndex2 = mapper.getDevice(2);
    auto * p2 = std::get<0>(devAtIndex2)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p2, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex2), localIndex1);

    ASSERT_EQ(p1, p2);

    ASSERT_TRUE(p1->getAxes(&axes2));
    ASSERT_EQ(axes2, 2);

    ASSERT_TRUE(p1->getRefPositionRaw(localIndex0, &ref0));
    ASSERT_NEAR(ref0, localIndex0, EPSILON);
    ASSERT_TRUE(p1->getRefPositionRaw(localIndex1, &ref1));
    ASSERT_NEAR(ref1, localIndex1, EPSILON);

    // device 3 [3-5]

    auto devAtIndex3 = mapper.getDevice(3);
    auto * p3 = std::get<0>(devAtIndex3)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p3, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex3), localIndex0);

    auto devAtIndex4 = mapper.getDevice(4);
    auto * p4 = std::get<0>(devAtIndex4)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p4, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex4), localIndex1);

    auto devAtIndex5 = mapper.getDevice(5);
    auto * p5 = std::get<0>(devAtIndex5)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p5, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex5), localIndex2);

    ASSERT_EQ(p3, p4);
    ASSERT_EQ(p3, p5);

    ASSERT_TRUE(p3->getAxes(&axes3));
    ASSERT_EQ(axes3, 3);

    ASSERT_TRUE(p3->getRefPositionRaw(localIndex0, &ref0));
    ASSERT_NEAR(ref0, localIndex0, EPSILON);
    ASSERT_TRUE(p3->getRefPositionRaw(localIndex1, &ref1));
    ASSERT_NEAR(ref1, localIndex1, EPSILON);
    ASSERT_TRUE(p3->getRefPositionRaw(localIndex2, &ref2));
    ASSERT_NEAR(ref2, localIndex2, EPSILON);

    // device 4 [6-9]

    auto devAtIndex6 = mapper.getDevice(6);
    auto * p6 = std::get<0>(devAtIndex6)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p6, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex6), localIndex0);

    auto devAtIndex7 = mapper.getDevice(7);
    auto * p7 = std::get<0>(devAtIndex7)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p7, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex7), localIndex1);

    auto devAtIndex8 = mapper.getDevice(8);
    auto * p8 = std::get<0>(devAtIndex8)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p8, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex8), localIndex2);

    auto devAtIndex9 = mapper.getDevice(9);
    auto * p9 = std::get<0>(devAtIndex9)->getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p9, nullptr);
    ASSERT_EQ(std::get<1>(devAtIndex9), localIndex3);

    ASSERT_EQ(p6, p7);
    ASSERT_EQ(p6, p8);
    ASSERT_EQ(p6, p9);

    ASSERT_TRUE(p6->getAxes(&axes4));
    ASSERT_EQ(axes4, 4);

    ASSERT_TRUE(p6->getRefPositionRaw(localIndex0, &ref0));
    ASSERT_NEAR(ref0, localIndex0, EPSILON);
    ASSERT_TRUE(p6->getRefPositionRaw(localIndex1, &ref1));
    ASSERT_NEAR(ref1, localIndex1, EPSILON);
    ASSERT_TRUE(p6->getRefPositionRaw(localIndex2, &ref2));
    ASSERT_NEAR(ref2, localIndex2, EPSILON);
    ASSERT_TRUE(p6->getRefPositionRaw(localIndex3, &ref3));
    ASSERT_NEAR(ref3, localIndex3, EPSILON);

    // DeviceMapper::getControlledAxes

    ASSERT_EQ(mapper.getControlledAxes(), axes1 + axes2 + axes3 + axes4);

    // DeviceMapper::getDevicesWithOffsets

    const auto & devicesWithOffsets = mapper.getDevicesWithOffsets();
    ASSERT_EQ(devicesWithOffsets.size(), 4);

    ASSERT_EQ(std::get<0>(devicesWithOffsets[0]), std::get<0>(devAtIndex0));
    ASSERT_EQ(std::get<0>(devicesWithOffsets[1]), std::get<0>(devAtIndex1));
    ASSERT_EQ(std::get<0>(devicesWithOffsets[2]), std::get<0>(devAtIndex3));
    ASSERT_EQ(std::get<0>(devicesWithOffsets[3]), std::get<0>(devAtIndex6));

    ASSERT_EQ(std::get<1>(devicesWithOffsets[0]), 0);
    ASSERT_EQ(std::get<1>(devicesWithOffsets[1]), 1);
    ASSERT_EQ(std::get<1>(devicesWithOffsets[2]), 3);
    ASSERT_EQ(std::get<1>(devicesWithOffsets[3]), 6);

    // DeviceMapper::getDevices

    const int globalAxesCount = 5;
    const int globalAxes[globalAxesCount] = {2, 4, 5, 6, 8};
    auto devices = mapper.getDevices(globalAxesCount, globalAxes);
    ASSERT_EQ(devices.size(), 3);

    ASSERT_EQ(std::get<0>(devices[0]), std::get<0>(devAtIndex2));
    ASSERT_EQ(std::get<0>(devices[1]), std::get<0>(devAtIndex4));
    ASSERT_EQ(std::get<0>(devices[1]), std::get<0>(devAtIndex5)); // same device
    ASSERT_EQ(std::get<0>(devices[2]), std::get<0>(devAtIndex6));
    ASSERT_EQ(std::get<0>(devices[2]), std::get<0>(devAtIndex8)); // same device

    ASSERT_EQ(std::get<1>(devices[0]).size(), 1);
    ASSERT_EQ(std::get<1>(devices[1]).size(), 2);
    ASSERT_EQ(std::get<1>(devices[2]).size(), 2);

    ASSERT_EQ(std::get<1>(devices[0]), (std::vector<int>{1})); // parens intentional
    ASSERT_EQ(std::get<1>(devices[1]), (std::vector<int>{1, 2}));
    ASSERT_EQ(std::get<1>(devices[2]), (std::vector<int>{0, 2}));

    ASSERT_EQ(std::get<2>(devices[0]), 0);
    ASSERT_EQ(std::get<2>(devices[1]), 1);
    ASSERT_EQ(std::get<2>(devices[2]), 3);

    // DeviceMapper::createTask

    auto task = mapper.createTask();
    ASSERT_EQ(task->size(), 0);

    // DeviceMapper::mapSingleJoint

    double ref_single;
    ASSERT_TRUE(mapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::getRefPositionRaw, 8, &ref_single));
    ASSERT_NEAR(ref_single, 2, EPSILON);

    // DeviceMapper::mapAllJoints

    double ref_full[10];
    ASSERT_TRUE(mapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, ref_full));
    ASSERT_EQ(std::vector<double>(ref_full, ref_full + 10), (std::vector<double>{0, 0, 1, 0, 1, 2, 0, 1, 2, 3})); // parens intentional

    // DeviceMapper::mapJointGroup

    const int jointCount = 5;
    const int joints[jointCount] = {1, 3, 5, 7, 9};
    double ref_group[jointCount];
    ASSERT_TRUE(mapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, jointCount, joints, ref_group));
    ASSERT_EQ(std::vector<double>(ref_group, ref_group + jointCount), (std::vector<double>{0, 0, 2, 1, 3})); // parens intentional
}

} // namespace roboticslab
