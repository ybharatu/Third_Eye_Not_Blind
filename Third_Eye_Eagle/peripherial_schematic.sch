<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.3.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="5" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="third_eye">
<packages>
<package name="MDBT40">
<wire x1="-5" y1="8" x2="5" y2="8" width="0.127" layer="21"/>
<wire x1="5" y1="8" x2="5" y2="4.25" width="0.127" layer="21"/>
<wire x1="5" y1="4.25" x2="5" y2="-8" width="0.127" layer="21"/>
<wire x1="5" y1="-8" x2="-5" y2="-8" width="0.127" layer="21"/>
<wire x1="-5" y1="-8" x2="-5" y2="4.25" width="0.127" layer="21"/>
<wire x1="-5" y1="4.25" x2="-5" y2="8" width="0.127" layer="21"/>
<wire x1="-5" y1="4.25" x2="5" y2="4.25" width="0.127" layer="21"/>
<smd name="P1" x="-4.65" y="3.8" dx="1.6" dy="0.9" layer="1"/>
<smd name="P25" x="4.65" y="-6.2" dx="1.6" dy="0.5" layer="1"/>
<smd name="P26" x="4.65" y="-5.5" dx="1.6" dy="0.5" layer="1"/>
<smd name="P27" x="4.65" y="-4.8" dx="1.6" dy="0.5" layer="1"/>
<smd name="P28" x="4.65" y="-4.1" dx="1.6" dy="0.5" layer="1"/>
<smd name="P11" x="-4.65" y="-5.5" dx="1.6" dy="0.5" layer="1"/>
<smd name="P10" x="-4.65" y="-4.8" dx="1.6" dy="0.5" layer="1"/>
<smd name="P9" x="-4.65" y="-4.1" dx="1.6" dy="0.5" layer="1"/>
<smd name="P8" x="-4.65" y="-3.4" dx="1.6" dy="0.5" layer="1"/>
<smd name="P7" x="-4.65" y="-2.7" dx="1.6" dy="0.5" layer="1"/>
<smd name="P6" x="-4.65" y="-2" dx="1.6" dy="0.5" layer="1"/>
<smd name="P5" x="-4.65" y="-1.3" dx="1.6" dy="0.5" layer="1"/>
<smd name="P4" x="-4.65" y="-0.6" dx="1.6" dy="0.5" layer="1"/>
<smd name="P3" x="-4.65" y="0.1" dx="1.6" dy="0.5" layer="1"/>
<smd name="P2" x="-4.65" y="0.8" dx="1.6" dy="0.5" layer="1"/>
<wire x1="-5.45" y1="8" x2="-5.45" y2="-8" width="0.01" layer="21"/>
<wire x1="5.45" y1="8.05" x2="5.45" y2="-7.95" width="0.01" layer="21"/>
<smd name="P39" x="4.65" y="3.8" dx="1.6" dy="0.9" layer="1"/>
<smd name="P29" x="4.65" y="-3.4" dx="1.6" dy="0.5" layer="1"/>
<smd name="P30" x="4.65" y="-2.7" dx="1.6" dy="0.5" layer="1"/>
<smd name="P31" x="4.65" y="-2" dx="1.6" dy="0.5" layer="1"/>
<smd name="P32" x="4.65" y="-1.3" dx="1.6" dy="0.5" layer="1"/>
<smd name="P33" x="4.65" y="-0.6" dx="1.6" dy="0.5" layer="1"/>
<smd name="P34" x="4.65" y="0.1" dx="1.6" dy="0.5" layer="1"/>
<smd name="P35" x="4.65" y="0.8" dx="1.6" dy="0.5" layer="1"/>
<smd name="P36" x="4.65" y="1.5" dx="1.6" dy="0.5" layer="1"/>
<smd name="P37" x="4.65" y="2.2" dx="1.6" dy="0.5" layer="1"/>
<smd name="P38" x="4.65" y="2.9" dx="1.6" dy="0.5" layer="1"/>
<smd name="P40" x="2.9" y="3.25" dx="1.2" dy="0.5" layer="1"/>
<smd name="P41" x="2.9" y="1.85" dx="1.2" dy="0.5" layer="1"/>
<smd name="P12" x="-4.2" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P13" x="-3.5" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P14" x="-2.8" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P15" x="-2.1" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P16" x="-1.4" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P17" x="-0.7" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P18" x="0" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P19" x="0.7" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P20" x="1.4" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P21" x="2.1" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P22" x="2.8" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P23" x="3.5" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<smd name="P24" x="4.2" y="-7.65" dx="1.6" dy="0.5" layer="1" rot="R90"/>
<polygon width="0.127" layer="41">
<vertex x="-5.4" y="8"/>
<vertex x="5.4" y="8"/>
<vertex x="5.4" y="6.105"/>
<vertex x="-5.4" y="6.105"/>
</polygon>
<polygon width="0.127" layer="41">
<vertex x="-0.5" y="2.262"/>
<vertex x="-0.5" y="4.073"/>
<vertex x="1.768" y="4.073"/>
<vertex x="1.768" y="2.262"/>
</polygon>
</package>
</packages>
<symbols>
<symbol name="MDBT40">
<pin name="P1-GND" x="-35.56" y="17.78" length="middle"/>
<pin name="P2-P0.25" x="-35.56" y="7.62" length="middle"/>
<pin name="P3-P0.26" x="-35.56" y="5.08" length="middle"/>
<pin name="P4-P0.27" x="-35.56" y="2.54" length="middle"/>
<pin name="P5-P0.28/AIN4" x="-35.56" y="0" length="middle"/>
<pin name="P6-P0.29/AIN5" x="-35.56" y="-2.54" length="middle"/>
<pin name="P7-P0.30/AIN6" x="-35.56" y="-5.08" length="middle"/>
<pin name="P8-P0.31/AIN7" x="-35.56" y="-7.62" length="middle"/>
<pin name="P9-DEC4" x="-35.56" y="-10.16" length="middle"/>
<pin name="P10-DCC" x="-35.56" y="-12.7" length="middle"/>
<pin name="P11-VDD" x="-35.56" y="-15.24" length="middle"/>
<pin name="P12-GND" x="-27.94" y="-45.72" length="middle" rot="R90"/>
<pin name="P13-P0.00/XL1" x="-25.4" y="-45.72" length="middle" rot="R90"/>
<pin name="P14-P0.01/XL2" x="-22.86" y="-45.72" length="middle" rot="R90"/>
<pin name="P15-P0.02/AIN0" x="-20.32" y="-45.72" length="middle" rot="R90"/>
<pin name="P16-P0.03/AIN1" x="-17.78" y="-45.72" length="middle" rot="R90"/>
<pin name="P17-P0.04/AIN2" x="-15.24" y="-45.72" length="middle" rot="R90"/>
<pin name="P18-P0.05/AIN3" x="-12.7" y="-45.72" length="middle" rot="R90"/>
<pin name="P19-P0.06" x="-10.16" y="-45.72" length="middle" rot="R90"/>
<pin name="P20-P0.07" x="-7.62" y="-45.72" length="middle" rot="R90"/>
<pin name="P21-P0.08" x="-5.08" y="-45.72" length="middle" rot="R90"/>
<pin name="P22-P0.09/NFC1" x="-2.54" y="-45.72" length="middle" rot="R90"/>
<pin name="P23-P0.10/NFC2" x="0" y="-45.72" length="middle" rot="R90"/>
<pin name="P24-GND" x="2.54" y="-45.72" length="middle" rot="R90"/>
<pin name="P25-P0.11" x="10.16" y="-15.24" length="middle" rot="R180"/>
<pin name="P26-P0.12" x="10.16" y="-12.7" length="middle" rot="R180"/>
<pin name="P27-P0.13" x="10.16" y="-10.16" length="middle" rot="R180"/>
<pin name="P28-P0.14" x="10.16" y="-7.62" length="middle" rot="R180"/>
<pin name="P29-P0.15" x="10.16" y="-5.08" length="middle" rot="R180"/>
<pin name="P30-P0.16" x="10.16" y="-2.54" length="middle" rot="R180"/>
<pin name="P31-P0.17" x="10.16" y="0" length="middle" rot="R180"/>
<pin name="P32-P0.18" x="10.16" y="2.54" length="middle" rot="R180"/>
<pin name="P33-P0.19" x="10.16" y="5.08" length="middle" rot="R180"/>
<pin name="P34-P0.20" x="10.16" y="7.62" length="middle" rot="R180"/>
<pin name="P35-P0.21/NRST" x="10.16" y="10.16" length="middle" rot="R180"/>
<pin name="P36-SWDCLK" x="10.16" y="12.7" length="middle" rot="R180"/>
<pin name="P37-SWDIO" x="10.16" y="15.24" length="middle" rot="R180"/>
<pin name="P38-P0.22" x="10.16" y="17.78" length="middle" rot="R180"/>
<pin name="P39-GND" x="10.16" y="20.32" length="middle" rot="R180"/>
<pin name="P40-P0.24" x="10.16" y="33.02" length="middle" rot="R180"/>
<pin name="P41-P0.23" x="10.16" y="30.48" length="middle" rot="R180"/>
<wire x1="5.08" y1="35.56" x2="5.08" y2="22.86" width="0.254" layer="94"/>
<wire x1="5.08" y1="22.86" x2="5.08" y2="-40.64" width="0.254" layer="94"/>
<wire x1="5.08" y1="-40.64" x2="-30.48" y2="-40.64" width="0.254" layer="94"/>
<wire x1="-30.48" y1="-40.64" x2="-30.48" y2="22.86" width="0.254" layer="94"/>
<wire x1="-30.48" y1="22.86" x2="-30.48" y2="35.56" width="0.254" layer="94"/>
<wire x1="-30.48" y1="35.56" x2="5.08" y2="35.56" width="0.254" layer="94"/>
<wire x1="-30.48" y1="22.86" x2="5.08" y2="22.86" width="0.254" layer="94"/>
<text x="-27.94" y="25.4" size="1.27" layer="94">MDBT42Q</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="MDBT42Q">
<gates>
<gate name="G$1" symbol="MDBT40" x="12.7" y="7.62"/>
</gates>
<devices>
<device name="" package="MDBT40">
<connects>
<connect gate="G$1" pin="P1-GND" pad="P1"/>
<connect gate="G$1" pin="P10-DCC" pad="P10"/>
<connect gate="G$1" pin="P11-VDD" pad="P11"/>
<connect gate="G$1" pin="P12-GND" pad="P12"/>
<connect gate="G$1" pin="P13-P0.00/XL1" pad="P13"/>
<connect gate="G$1" pin="P14-P0.01/XL2" pad="P14"/>
<connect gate="G$1" pin="P15-P0.02/AIN0" pad="P15"/>
<connect gate="G$1" pin="P16-P0.03/AIN1" pad="P16"/>
<connect gate="G$1" pin="P17-P0.04/AIN2" pad="P17"/>
<connect gate="G$1" pin="P18-P0.05/AIN3" pad="P18"/>
<connect gate="G$1" pin="P19-P0.06" pad="P19"/>
<connect gate="G$1" pin="P2-P0.25" pad="P2"/>
<connect gate="G$1" pin="P20-P0.07" pad="P20"/>
<connect gate="G$1" pin="P21-P0.08" pad="P21"/>
<connect gate="G$1" pin="P22-P0.09/NFC1" pad="P22"/>
<connect gate="G$1" pin="P23-P0.10/NFC2" pad="P23"/>
<connect gate="G$1" pin="P24-GND" pad="P24"/>
<connect gate="G$1" pin="P25-P0.11" pad="P25"/>
<connect gate="G$1" pin="P26-P0.12" pad="P26"/>
<connect gate="G$1" pin="P27-P0.13" pad="P27"/>
<connect gate="G$1" pin="P28-P0.14" pad="P28"/>
<connect gate="G$1" pin="P29-P0.15" pad="P29"/>
<connect gate="G$1" pin="P3-P0.26" pad="P3"/>
<connect gate="G$1" pin="P30-P0.16" pad="P30"/>
<connect gate="G$1" pin="P31-P0.17" pad="P31"/>
<connect gate="G$1" pin="P32-P0.18" pad="P32"/>
<connect gate="G$1" pin="P33-P0.19" pad="P33"/>
<connect gate="G$1" pin="P34-P0.20" pad="P34"/>
<connect gate="G$1" pin="P35-P0.21/NRST" pad="P35"/>
<connect gate="G$1" pin="P36-SWDCLK" pad="P36"/>
<connect gate="G$1" pin="P37-SWDIO" pad="P37"/>
<connect gate="G$1" pin="P38-P0.22" pad="P38"/>
<connect gate="G$1" pin="P39-GND" pad="P39"/>
<connect gate="G$1" pin="P4-P0.27" pad="P4"/>
<connect gate="G$1" pin="P40-P0.24" pad="P40"/>
<connect gate="G$1" pin="P41-P0.23" pad="P41"/>
<connect gate="G$1" pin="P5-P0.28/AIN4" pad="P5"/>
<connect gate="G$1" pin="P6-P0.29/AIN5" pad="P6"/>
<connect gate="G$1" pin="P7-P0.30/AIN6" pad="P7"/>
<connect gate="G$1" pin="P8-P0.31/AIN7" pad="P8"/>
<connect gate="G$1" pin="P9-DEC4" pad="P9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="third_eye" deviceset="MDBT42Q" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="58.42" y="53.34" smashed="yes"/>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
