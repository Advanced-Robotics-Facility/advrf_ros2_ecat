#include <advrf_ros2_ecat/converter.hpp>
#include <gtest/gtest.h>

using namespace advrf_ros2_ecat;

namespace
{
Commands command()
{
    Commands msg;
    msg.header.stamp.sec = 123;
    msg.ecat_id = {7};
    msg.pos_ref = {0.5f};
    msg.vel_ref = {0};
    msg.tor_ref = {0};
    msg.cur_ref = {0};
    msg.gain_0 = {1};
    msg.gain_1 = {0};
    msg.gain_2 = {0};
    msg.gain_3 = {0};
    msg.gain_4 = {0};
    msg.fault_ack = {0};
    msg.ts = {0};
    msg.op_idx_aux = {0};
    msg.aux = {0};
    return msg;
}

MotorReferenceMap references()
{
    MotorPdoTx::pdo_t value{};
    std::get<0>(value) = iit::advr::Gains_Type_POSITION;
    return {{7, value}, {2, value}};
}
}

TEST(Converter, PartialCommandPreservesOtherMotors)
{
    const auto previous = references();
    const auto converted = convert::from_ros(command(), previous);
    ASSERT_TRUE(converted);
    EXPECT_FLOAT_EQ(std::get<1>(converted->at(7)), 0.5f);
    EXPECT_EQ(converted->at(2), previous.at(2));
}

TEST(Converter, RejectsEveryMismatchedArray)
{
    const auto previous = references();
    const auto valid = command();
#define CHECK_ARRAY(field) \
    { auto msg = valid; msg.field.clear(); EXPECT_FALSE(convert::from_ros(msg, previous)) << #field; } \
    { auto msg = valid; msg.field.push_back(0); EXPECT_FALSE(convert::from_ros(msg, previous)) << #field; }
    CHECK_ARRAY(pos_ref)
    CHECK_ARRAY(vel_ref)
    CHECK_ARRAY(tor_ref)
    CHECK_ARRAY(cur_ref)
    CHECK_ARRAY(gain_0)
    CHECK_ARRAY(gain_1)
    CHECK_ARRAY(gain_2)
    CHECK_ARRAY(gain_3)
    CHECK_ARRAY(gain_4)
    CHECK_ARRAY(fault_ack)
    CHECK_ARRAY(ts)
    CHECK_ARRAY(op_idx_aux)
    CHECK_ARRAY(aux)
#undef CHECK_ARRAY
}

TEST(Converter, PositionGainsPassThroughUnchanged)
{
    auto msg = command();
    msg.gain_0 = {1.f};
    msg.gain_1 = {2.f};
    msg.gain_2 = {3.f};
    const auto converted = convert::from_ros(msg, references());
    ASSERT_TRUE(converted);
    const auto& pdo = converted->at(7);
    EXPECT_FLOAT_EQ(std::get<4>(pdo), 1.f);
    EXPECT_FLOAT_EQ(std::get<5>(pdo), 2.f);
    EXPECT_FLOAT_EQ(std::get<6>(pdo), 3.f);
    EXPECT_FLOAT_EQ(std::get<7>(pdo), 0.f);
    EXPECT_FLOAT_EQ(std::get<8>(pdo), 0.f);
}

TEST(Converter, PositionAndVelocityRejectUnusedGainSlots)
{
    for (int mode : {iit::advr::Gains_Type_POSITION, iit::advr::Gains_Type_VELOCITY}) {
        MotorPdoTx::pdo_t value{};
        std::get<0>(value) = mode;
        const MotorReferenceMap previous{{7, value}};

        auto msg = command();
        msg.gain_3 = {1.f};
        EXPECT_FALSE(convert::from_ros(msg, previous)) << "gain_3 mode " << mode;

        msg = command();
        msg.gain_4 = {1.f};
        EXPECT_FALSE(convert::from_ros(msg, previous)) << "gain_4 mode " << mode;
    }
}

TEST(Converter, ImpedanceKeepsAllFiveGains)
{
    MotorPdoTx::pdo_t value{};
    std::get<0>(value) = iit::advr::Gains_Type_IMPEDANCE;
    auto msg = command();
    msg.gain_0 = {1.f};
    msg.gain_1 = {2.f};
    msg.gain_2 = {3.f};
    msg.gain_3 = {4.f};
    msg.gain_4 = {5.f};
    const auto converted = convert::from_ros(msg, {{7, value}});
    ASSERT_TRUE(converted);
    const auto& pdo = converted->at(7);
    EXPECT_FLOAT_EQ(std::get<4>(pdo), 1.f);
    EXPECT_FLOAT_EQ(std::get<5>(pdo), 2.f);
    EXPECT_FLOAT_EQ(std::get<6>(pdo), 3.f);
    EXPECT_FLOAT_EQ(std::get<7>(pdo), 4.f);
    EXPECT_FLOAT_EQ(std::get<8>(pdo), 5.f);
}

TEST(Converter, RejectsEmptyAndUnknownIds)
{
    EXPECT_FALSE(convert::from_ros(Commands{}, references()));
    auto msg = command();
    msg.ecat_id[0] = 99;
    EXPECT_FALSE(convert::from_ros(msg, references()));
}
