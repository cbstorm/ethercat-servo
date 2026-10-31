// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "ethercat_motion/esi_parser.hpp"
#include "test_helpers.hpp"

using namespace ethercat_motion;
using ethercat_motion::test::sourcePath;

TEST(Esi, ParsesVendorFileAndIslDevice) {
    auto esi = parseEsiFile(sourcePath("esi/LC-EH_V1.5.xml"));
    ASSERT_TRUE(esi.isOk()) << esi.status().toString();
    EXPECT_EQ(esi->vendor_id, 0x00000A79U);
    EXPECT_EQ(esi->devices.size(), 10U);

    const EsiDevice* isl = esi->find(0x5800, 0x0001);
    ASSERT_NE(isl, nullptr);
    EXPECT_EQ(isl->name, "ISL-EC");
    EXPECT_EQ(isl->profile, 402U);
    EXPECT_EQ(esi->find(0x5800, 0x0002), nullptr);

    // Default assignment (0x1C12/0x1C13): only PDOs carrying an Sm attribute.
    const auto syncs = isl->defaultSyncConfig();
    ASSERT_EQ(syncs.size(), 2U);
    EXPECT_EQ(syncs[0].index, 2);
    EXPECT_EQ(syncs[0].direction, PdoDirection::Output);
    ASSERT_EQ(syncs[0].pdos.size(), 1U);
    EXPECT_EQ(syncs[0].pdos[0].index, 0x1600);
    ASSERT_EQ(syncs[0].pdos[0].entries.size(), 5U);
    EXPECT_EQ(syncs[0].pdos[0].entries[0].index, 0x6040);
    EXPECT_EQ(syncs[0].pdos[0].entries[1].index, 0x607A);
    EXPECT_EQ(syncs[0].pdos[0].entries[2].index, 0x60FF);
    EXPECT_EQ(syncs[0].pdos[0].entries[4].index, 0x60FE);
    EXPECT_EQ(syncs[0].pdos[0].entries[4].subindex, 1);
    EXPECT_EQ(syncs[1].index, 3);
    ASSERT_EQ(syncs[1].pdos.size(), 1U);
    EXPECT_EQ(syncs[1].pdos[0].index, 0x1A00);
    EXPECT_EQ(syncs[1].pdos[0].entries.size(), 12U);

    // 0x1601 (modes of operation) exists but is NOT assigned by default.
    ASSERT_EQ(isl->rx_pdos.size(), 2U);
    EXPECT_EQ(isl->rx_pdos[1].index, 0x1601);
    EXPECT_FALSE(isl->rx_pdos[1].sm.has_value());

    // InitCmd PS: 0x6060 = 8 (CSP)
    const auto sdos = isl->startupSdos();
    ASSERT_EQ(sdos.size(), 1U);
    EXPECT_EQ(sdos[0].index, 0x6060);
    EXPECT_EQ(sdos[0].size, 1);
    EXPECT_EQ(sdos[0].value, 8);

    ASSERT_TRUE(isl->dcAssignActivate().has_value());
    EXPECT_EQ(*isl->dcAssignActivate(), 0x0300);

    EXPECT_TRUE(isl->isMappable(0x6071, PdoDirection::Output));
    EXPECT_TRUE(isl->isMappable(0x6077, PdoDirection::Input));
    EXPECT_FALSE(isl->isMappable(0x6064, PdoDirection::Output));
    // 0x6502 = 0x1DD: PP, PV, HM, CSP, CSV - but NOT CST (bit 9).
    ASSERT_TRUE(isl->supportedDriveModes().has_value());
    EXPECT_EQ(*isl->supportedDriveModes(), 0x1DDU);
    EXPECT_EQ(*isl->supportedDriveModes() & (1U << 9), 0U);
}

TEST(Esi, RejectsCanopenEds) {
    auto esi = parseEsiFile(sourcePath("../ISL_C_V1.0.eds"));
    if (!esi.isOk() && esi.status().code() == ErrorCode::IoError) {
        GTEST_SKIP() << "EDS reference file not present";
    }
    EXPECT_FALSE(esi.isOk());
}

TEST(Esi, NotAnEsiDocument) {
    auto esi = parseEsiString("<?xml version=\"1.0\"?><Foo/>");
    ASSERT_FALSE(esi.isOk());
    EXPECT_NE(esi.status().message().find("EtherCATInfo"), std::string::npos);
}

TEST(Esi, MalformedXml) {
    EXPECT_FALSE(parseEsiString("<EtherCATInfo><Vendor></EtherCATInfo>").isOk());
    EXPECT_FALSE(parseEsiString("").isOk());
}

TEST(Esi, EntitiesCdataAndNumbers) {
    const char* xml = R"(<?xml version="1.0"?>
<!-- comment -->
<EtherCATInfo>
  <Vendor><Id>#x00000002</Id><Name>A &amp; B</Name></Vendor>
  <Descriptions><Devices>
    <Device>
      <Type ProductCode="#x10" RevisionNo="0x2">T</Type>
      <Name><![CDATA[Dev <1>]]></Name>
      <RxPdo Sm="2" Fixed="1"><Index>#x1600</Index><Name>Out</Name>
        <Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen></Entry>
      </RxPdo>
      <Mailbox><CoE><InitCmd><Transition>PS</Transition><Index>#x6083</Index><SubIndex>0</SubIndex><Data>10270000</Data></InitCmd></CoE></Mailbox>
    </Device>
  </Devices></Descriptions>
</EtherCATInfo>)";
    auto esi = parseEsiString(xml);
    ASSERT_TRUE(esi.isOk()) << esi.status().toString();
    EXPECT_EQ(esi->vendor_name, "A & B");
    ASSERT_EQ(esi->devices.size(), 1U);
    EXPECT_EQ(esi->devices[0].name, "Dev <1>");
    EXPECT_EQ(esi->devices[0].product_code, 0x10U);
    EXPECT_EQ(esi->devices[0].revision, 2U);
    EXPECT_TRUE(esi->devices[0].rx_pdos[0].fixed);
    const auto sdos = esi->devices[0].startupSdos();
    ASSERT_EQ(sdos.size(), 1U);
    EXPECT_EQ(sdos[0].size, 4);
    EXPECT_EQ(sdos[0].value, 10000);  // little endian 0x00002710
}

TEST(Esi, YamlExport) {
    auto esi = parseEsiFile(sourcePath("esi/LC-EH_V1.5.xml"));
    ASSERT_TRUE(esi.isOk());
    const std::string yaml = esiDeviceToYaml(*esi->find(0x5800));
    EXPECT_NE(yaml.find("product_code: 0x00005800"), std::string::npos);
    EXPECT_NE(yaml.find("- index: 0x1600"), std::string::npos);
    EXPECT_NE(yaml.find("[0x6040, 0x00, 16]"), std::string::npos);
    EXPECT_NE(yaml.find("assign_activate: 0x0300"), std::string::npos);
}

TEST(Esi, ParseNumbers) {
    EXPECT_EQ(parseEsiNumber("#x1A00"), 0x1A00U);
    EXPECT_EQ(parseEsiNumber("0x10"), 16U);
    EXPECT_EQ(parseEsiNumber(" 42 "), 42U);
    EXPECT_FALSE(parseEsiNumber("x").has_value());
    EXPECT_FALSE(parseEsiNumber("").has_value());
}
