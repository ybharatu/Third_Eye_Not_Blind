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
<layer number="2" name="Route2" color="16" fill="1" visible="no" active="no"/>
<layer number="3" name="Route3" color="17" fill="1" visible="no" active="no"/>
<layer number="4" name="Route4" color="18" fill="1" visible="no" active="no"/>
<layer number="5" name="Route5" color="19" fill="1" visible="no" active="no"/>
<layer number="6" name="Route6" color="25" fill="1" visible="no" active="no"/>
<layer number="7" name="Route7" color="26" fill="1" visible="no" active="no"/>
<layer number="8" name="Route8" color="27" fill="1" visible="no" active="no"/>
<layer number="9" name="Route9" color="28" fill="1" visible="no" active="no"/>
<layer number="10" name="Route10" color="29" fill="1" visible="no" active="no"/>
<layer number="11" name="Route11" color="30" fill="1" visible="no" active="no"/>
<layer number="12" name="Route12" color="20" fill="1" visible="no" active="no"/>
<layer number="13" name="Route13" color="21" fill="1" visible="no" active="no"/>
<layer number="14" name="Route14" color="22" fill="1" visible="no" active="no"/>
<layer number="15" name="Route15" color="23" fill="1" visible="no" active="no"/>
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
<package name="MA10-1" urn="urn:adsk.eagle:footprint:8300/1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-12.065" y1="1.27" x2="-10.795" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="1.27" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-0.635" x2="-10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="1.27" x2="-8.255" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.27" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-0.635" x2="-8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-1.27" x2="-9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="-1.27" x2="-10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="0.635" x2="-12.7" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="1.27" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="-0.635" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="-1.27" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="1.27" x2="-5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-1.27" x2="-6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-1.27" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.27" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-1.27" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="1.27" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-0.635" x2="9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0.635" x2="10.795" y2="1.27" width="0.1524" layer="21"/>
<wire x1="10.795" y1="1.27" x2="12.065" y2="1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="1.27" x2="12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-0.635" x2="12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="-1.27" x2="10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="10.795" y1="-1.27" x2="10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="-1.27" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="12.7" y1="0.635" x2="12.7" y2="-0.635" width="0.1524" layer="21"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="10" x="11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-12.7" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-11.938" y="-2.921" size="1.27" layer="21" ratio="10">1</text>
<text x="10.795" y="1.651" size="1.27" layer="21" ratio="10">10</text>
<text x="1.27" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-9.144" y1="-0.254" x2="-8.636" y2="0.254" layer="51"/>
<rectangle x1="-11.684" y1="-0.254" x2="-11.176" y2="0.254" layer="51"/>
<rectangle x1="-6.604" y1="-0.254" x2="-6.096" y2="0.254" layer="51"/>
<rectangle x1="-4.064" y1="-0.254" x2="-3.556" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="3.556" y1="-0.254" x2="4.064" y2="0.254" layer="51"/>
<rectangle x1="6.096" y1="-0.254" x2="6.604" y2="0.254" layer="51"/>
<rectangle x1="11.176" y1="-0.254" x2="11.684" y2="0.254" layer="51"/>
<rectangle x1="8.636" y1="-0.254" x2="9.144" y2="0.254" layer="51"/>
</package>
<package name="MA03-2" urn="urn:adsk.eagle:footprint:8265/1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="-1.27" drill="1.016"/>
<pad name="3" x="0" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="5" x="2.54" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="2" x="-2.54" y="1.27" drill="1.016" shape="octagon"/>
<pad name="4" x="0" y="1.27" drill="1.016" shape="octagon"/>
<pad name="6" x="2.54" y="1.27" drill="1.016" shape="octagon"/>
<text x="-3.175" y="-4.191" size="1.27" layer="21" ratio="10">1</text>
<text x="-3.81" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="4.064" y="0.635" size="1.27" layer="21" ratio="10">6</text>
<text x="-1.27" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
</package>
<package name="MOLEX_0473460001">
<wire x1="-4.2" y1="-4.44" x2="4.2" y2="-4.44" width="0.127" layer="21"/>
<wire x1="-3.9" y1="-5.14" x2="3.9" y2="-5.14" width="0.127" layer="21"/>
<wire x1="-4.5" y1="-5.5" x2="4.5" y2="-5.5" width="0.127" layer="39"/>
<wire x1="4.5" y1="-5.5" x2="4.5" y2="1" width="0.127" layer="39"/>
<wire x1="4.5" y1="1" x2="-4.5" y2="1" width="0.127" layer="39"/>
<wire x1="-4.5" y1="1" x2="-4.5" y2="-5.5" width="0.127" layer="39"/>
<text x="-4.50605" y="1.25168125" size="1.271709375" layer="25">&gt;NAME</text>
<text x="-4.5023" y="-7.00358125" size="1.27065" layer="27">&gt;VALUE</text>
<circle x="-4" y="0.5" radius="0.25" width="0" layer="21"/>
<smd name="3" x="0" y="0" dx="0.45" dy="1.38" layer="1"/>
<smd name="2" x="-0.65" y="0" dx="0.45" dy="1.38" layer="1"/>
<smd name="4" x="0.65" y="0" dx="0.45" dy="1.38" layer="1"/>
<smd name="1" x="-1.3" y="0" dx="0.45" dy="1.38" layer="1"/>
<smd name="5" x="1.3" y="0" dx="0.45" dy="1.38" layer="1"/>
<smd name="P1" x="-2.4625" y="-0.36" dx="1.475" dy="2.1" layer="1"/>
<smd name="P2" x="2.4625" y="-0.36" dx="1.475" dy="2.1" layer="1"/>
<smd name="P3" x="-2.9125" y="-2.99" dx="2.375" dy="1.9" layer="1"/>
<smd name="P6" x="2.9125" y="-2.99" dx="2.375" dy="1.9" layer="1"/>
<smd name="P4" x="-0.8375" y="-2.99" dx="1.175" dy="1.9" layer="1"/>
<smd name="P5" x="0.8375" y="-2.99" dx="1.175" dy="1.9" layer="1"/>
</package>
<package name="SOT95P280X145-5N">
<wire x1="-0.88" y1="1.53" x2="-0.88" y2="-1.53" width="0.127" layer="51"/>
<wire x1="-0.88" y1="-1.53" x2="0.88" y2="-1.53" width="0.127" layer="51"/>
<wire x1="0.88" y1="-1.53" x2="0.88" y2="1.53" width="0.127" layer="51"/>
<wire x1="0.88" y1="1.53" x2="-0.88" y2="1.53" width="0.127" layer="51"/>
<wire x1="-0.88" y1="1.53" x2="0.88" y2="1.53" width="0.127" layer="21"/>
<wire x1="-0.88" y1="-1.53" x2="0.88" y2="-1.53" width="0.127" layer="21"/>
<text x="-2.0087" y="2.05891875" size="1.02041875" layer="25">&gt;NAME</text>
<text x="-2.05626875" y="-2.908859375" size="1.019109375" layer="27">&gt;VALUE</text>
<wire x1="-2.11" y1="1.5" x2="-1.13" y2="1.5" width="0.05" layer="39"/>
<wire x1="-1.13" y1="1.5" x2="-1.13" y2="1.78" width="0.05" layer="39"/>
<wire x1="-1.13" y1="1.78" x2="1.13" y2="1.78" width="0.05" layer="39"/>
<wire x1="1.13" y1="1.78" x2="1.13" y2="1.5" width="0.05" layer="39"/>
<wire x1="1.13" y1="1.5" x2="2.11" y2="1.5" width="0.05" layer="39"/>
<wire x1="2.11" y1="1.5" x2="2.11" y2="-1.5" width="0.05" layer="39"/>
<wire x1="2.11" y1="-1.5" x2="1.13" y2="-1.5" width="0.05" layer="39"/>
<wire x1="1.13" y1="-1.5" x2="1.13" y2="-1.78" width="0.05" layer="39"/>
<wire x1="1.13" y1="-1.78" x2="-1.13" y2="-1.78" width="0.05" layer="39"/>
<wire x1="-1.13" y1="-1.78" x2="-1.13" y2="-1.5" width="0.05" layer="39"/>
<wire x1="-1.13" y1="-1.5" x2="-2.11" y2="-1.5" width="0.05" layer="39"/>
<wire x1="-2.11" y1="-1.5" x2="-2.11" y2="1.5" width="0.05" layer="39"/>
<circle x="-2.45" y="1.05" radius="0.1581125" width="0.2" layer="21"/>
<smd name="1" x="-1.255" y="0.95" dx="1.21" dy="0.59" layer="1" roundness="50"/>
<smd name="2" x="-1.255" y="0" dx="1.21" dy="0.59" layer="1" roundness="50"/>
<smd name="3" x="-1.255" y="-0.95" dx="1.21" dy="0.59" layer="1" roundness="50"/>
<smd name="4" x="1.255" y="-0.95" dx="1.21" dy="0.59" layer="1" roundness="50" rot="R180"/>
<smd name="5" x="1.255" y="0.95" dx="1.21" dy="0.59" layer="1" roundness="50" rot="R180"/>
</package>
<package name="M1206" urn="urn:adsk.eagle:footprint:23066/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="PN87520" urn="urn:adsk.eagle:footprint:6141/1">
<description>&lt;b&gt;USB connector&lt;/b&gt;</description>
<wire x1="-7.4" y1="-10.19" x2="7.4" y2="-10.19" width="0.254" layer="21"/>
<wire x1="7.4" y1="-10.19" x2="7.4" y2="4.11" width="0.254" layer="21"/>
<wire x1="7.4" y1="4.11" x2="-7.4" y2="4.11" width="0.254" layer="21"/>
<wire x1="-7.4" y1="4.11" x2="-7.4" y2="-10.19" width="0.254" layer="21"/>
<wire x1="-5.08" y1="-2.87" x2="-3.81" y2="-8.72" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-8.72" x2="-2.54" y2="-8.72" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-8.72" x2="-1.27" y2="-2.87" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-2.87" x2="2.54" y2="-8.72" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-8.72" x2="3.81" y2="-8.72" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-8.72" x2="5.08" y2="-2.87" width="0.1524" layer="21"/>
<wire x1="-2.46" y1="-0.1" x2="-2.46" y2="0.9" width="0.0508" layer="21" curve="180"/>
<wire x1="-2.46" y1="-1.1" x2="-2.46" y2="-0.1" width="0.0508" layer="21" curve="180"/>
<wire x1="3.665" y1="0.4" x2="3.665" y2="-0.6" width="0.0508" layer="21" curve="180"/>
<wire x1="3.415" y1="0.9" x2="3.415" y2="-1.1" width="0.0508" layer="21" curve="180"/>
<wire x1="3.665" y1="0.4" x2="4.165" y2="0.4" width="0.0508" layer="21" curve="-15.189287"/>
<wire x1="3.415" y1="0.9" x2="4.175" y2="0.845" width="0.0508" layer="21" curve="-12.05913"/>
<wire x1="3.415" y1="-1.1" x2="4.165" y2="-0.975" width="0.0508" layer="21" curve="18.422836"/>
<wire x1="1.665" y1="-0.35" x2="1.665" y2="0.9" width="0.0508" layer="21" curve="180"/>
<wire x1="1.29" y1="0.025" x2="1.29" y2="0.4" width="0.0508" layer="21" curve="180"/>
<wire x1="-3.835" y1="0.9" x2="-3.835" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="-3.835" y1="-1.1" x2="-3.21" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="-3.21" y1="-1.1" x2="-3.21" y2="-0.1" width="0.0508" layer="21"/>
<wire x1="-3.21" y1="-0.1" x2="-3.21" y2="0.9" width="0.0508" layer="21"/>
<wire x1="-3.21" y1="0.9" x2="-3.835" y2="0.9" width="0.0508" layer="21"/>
<wire x1="-3.21" y1="0.9" x2="-2.46" y2="0.4" width="0.0508" layer="21"/>
<wire x1="-2.46" y1="0.4" x2="-3.21" y2="-0.1" width="0.0508" layer="21"/>
<wire x1="-3.21" y1="-0.1" x2="-2.46" y2="-0.6" width="0.0508" layer="21"/>
<wire x1="-2.46" y1="-0.6" x2="-3.21" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="-2.46" y1="-1.1" x2="-3.21" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="-2.46" y1="0.9" x2="-3.21" y2="0.9" width="0.0508" layer="21"/>
<wire x1="-1.71" y1="0.9" x2="-1.71" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="-1.71" y1="-1.1" x2="0.04" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="0.04" y1="-1.1" x2="0.04" y2="-0.6" width="0.0508" layer="21"/>
<wire x1="0.04" y1="-0.6" x2="-1.085" y2="-0.6" width="0.0508" layer="21"/>
<wire x1="-1.085" y1="-0.6" x2="-1.085" y2="-0.35" width="0.0508" layer="21"/>
<wire x1="-1.085" y1="-0.35" x2="0.04" y2="-0.35" width="0.0508" layer="21"/>
<wire x1="0.04" y1="-0.35" x2="0.04" y2="0.15" width="0.0508" layer="21"/>
<wire x1="0.04" y1="0.15" x2="-1.085" y2="0.15" width="0.0508" layer="21"/>
<wire x1="-1.085" y1="0.15" x2="-1.085" y2="0.4" width="0.0508" layer="21"/>
<wire x1="-1.085" y1="0.4" x2="0.04" y2="0.4" width="0.0508" layer="21"/>
<wire x1="0.04" y1="0.4" x2="0.04" y2="0.9" width="0.0508" layer="21"/>
<wire x1="0.04" y1="0.9" x2="-1.71" y2="0.9" width="0.0508" layer="21"/>
<wire x1="0.29" y1="0.9" x2="0.29" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="0.29" y1="-1.1" x2="0.915" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="0.915" y1="-1.1" x2="0.915" y2="-0.35" width="0.0508" layer="21"/>
<wire x1="0.915" y1="-0.35" x2="1.415" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="1.415" y1="-1.1" x2="2.165" y2="-1.1" width="0.0508" layer="21"/>
<wire x1="2.165" y1="-1.1" x2="1.665" y2="-0.35" width="0.0508" layer="21"/>
<wire x1="0.915" y1="0.4" x2="0.915" y2="0.025" width="0.0508" layer="21"/>
<wire x1="0.29" y1="0.9" x2="1.665" y2="0.9" width="0.0508" layer="21"/>
<wire x1="0.915" y1="0.4" x2="1.29" y2="0.4" width="0.0508" layer="21"/>
<wire x1="0.915" y1="0.025" x2="1.29" y2="0.025" width="0.0508" layer="21"/>
<wire x1="3.665" y1="-0.1" x2="4.165" y2="-0.1" width="0.0508" layer="21"/>
<wire x1="3.665" y1="-0.1" x2="3.665" y2="-0.6" width="0.0508" layer="21"/>
<wire x1="4.16" y1="0.4" x2="4.16" y2="0.845" width="0.0508" layer="21"/>
<wire x1="4.165" y1="-0.1" x2="4.165" y2="-0.975" width="0.0508" layer="21"/>
<pad name="2" x="-1" y="2.71" drill="0.95" shape="long" rot="R90"/>
<pad name="3" x="1" y="2.71" drill="0.95" shape="long" rot="R90"/>
<pad name="1" x="-3.5" y="2.71" drill="0.95" shape="long" rot="R90"/>
<pad name="4" x="3.5" y="2.71" drill="0.95" shape="long" rot="R90"/>
<text x="-7.62" y="5.08" size="1.27" layer="25">&gt;NAME</text>
<text x="9.144" y="-10.16" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-3.81" y="-2.04" size="0.4064" layer="21">E L E C T R O N I C S</text>
<hole x="-6.57" y="0" drill="2.3"/>
<hole x="6.57" y="0" drill="2.3"/>
</package>
<package name="C1206" urn="urn:adsk.eagle:footprint:23125/1">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="MA07-2" urn="urn:adsk.eagle:footprint:8279/1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-8.255" y1="2.54" x2="-6.985" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="2.54" x2="-6.35" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="1.905" x2="-5.715" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="2.54" x2="-4.445" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="2.54" x2="-8.89" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="4.445" y2="2.54" width="0.1524" layer="21"/>
<wire x1="4.445" y1="2.54" x2="5.715" y2="2.54" width="0.1524" layer="21"/>
<wire x1="5.715" y1="2.54" x2="6.35" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="-1.905" x2="-6.985" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-4.445" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="-2.54" x2="-5.715" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-2.54" x2="-6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="1.905" x2="-8.89" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-1.905" x2="-8.255" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-2.54" x2="-8.255" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-2.54" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.35" y1="-1.905" x2="5.715" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="5.715" y1="-2.54" x2="4.445" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-2.54" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="8.89" y1="1.905" x2="8.89" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="8.255" y1="2.54" x2="8.89" y2="1.905" width="0.1524" layer="21"/>
<wire x1="6.985" y1="2.54" x2="8.255" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.35" y1="1.905" x2="6.985" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-2.54" x2="6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.54" x2="6.985" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.89" y1="-1.905" x2="8.255" y2="-2.54" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="3" x="-5.08" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="5" x="-2.54" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="7" x="0" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="9" x="2.54" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="11" x="5.08" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="2" x="-7.62" y="1.27" drill="1.016" shape="octagon"/>
<pad name="4" x="-5.08" y="1.27" drill="1.016" shape="octagon"/>
<pad name="6" x="-2.54" y="1.27" drill="1.016" shape="octagon"/>
<pad name="8" x="0" y="1.27" drill="1.016" shape="octagon"/>
<pad name="10" x="2.54" y="1.27" drill="1.016" shape="octagon"/>
<pad name="12" x="5.08" y="1.27" drill="1.016" shape="octagon"/>
<pad name="13" x="7.62" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="14" x="7.62" y="1.27" drill="1.016" shape="octagon"/>
<text x="-8.128" y="-4.191" size="1.27" layer="21" ratio="10">1</text>
<text x="-8.89" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="6.477" y="2.921" size="1.27" layer="21" ratio="10">14</text>
<text x="-2.54" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.334" y1="-1.524" x2="-4.826" y2="-1.016" layer="51"/>
<rectangle x1="-7.874" y1="-1.524" x2="-7.366" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="4.826" y1="-1.524" x2="5.334" y2="-1.016" layer="51"/>
<rectangle x1="-7.874" y1="1.016" x2="-7.366" y2="1.524" layer="51"/>
<rectangle x1="-5.334" y1="1.016" x2="-4.826" y2="1.524" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
<rectangle x1="4.826" y1="1.016" x2="5.334" y2="1.524" layer="51"/>
<rectangle x1="7.366" y1="1.016" x2="7.874" y2="1.524" layer="51"/>
<rectangle x1="7.366" y1="-1.524" x2="7.874" y2="-1.016" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="MA10-1" urn="urn:adsk.eagle:package:8346/1" type="box">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA10-1"/>
</packageinstances>
</package3d>
<package3d name="MA03-2" urn="urn:adsk.eagle:package:8334/1" type="box">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA03-2"/>
</packageinstances>
</package3d>
<package3d name="PN87520" urn="urn:adsk.eagle:package:6146/1" type="box">
<description>USB connector</description>
<packageinstances>
<packageinstance name="PN87520"/>
</packageinstances>
</package3d>
<package3d name="MA07-2" urn="urn:adsk.eagle:package:8330/1" type="box">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA07-2"/>
</packageinstances>
</package3d>
</packages3d>
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
<symbol name="MA10-1">
<wire x1="3.81" y1="-12.7" x2="-8.89" y2="-12.7" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="2.54" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-7.62" x2="2.54" y2="-7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-10.16" x2="2.54" y2="-10.16" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="7.62" x2="2.54" y2="7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="5.08" x2="2.54" y2="5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="12.7" x2="2.54" y2="12.7" width="0.6096" layer="94"/>
<wire x1="1.27" y1="10.16" x2="2.54" y2="10.16" width="0.6096" layer="94"/>
<wire x1="-8.89" y1="15.24" x2="-8.89" y2="-12.7" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-12.7" x2="3.81" y2="15.24" width="0.4064" layer="94"/>
<wire x1="-8.89" y1="15.24" x2="3.81" y2="15.24" width="0.4064" layer="94"/>
<text x="-1.27" y="-15.24" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="16.002" size="1.778" layer="95">&gt;NAME</text>
<pin name="GND" x="7.62" y="-10.16" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="VIN" x="7.62" y="-7.62" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="D/C" x="7.62" y="-5.08" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="RST" x="7.62" y="-2.54" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="SDCS" x="7.62" y="0" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="CS" x="7.62" y="2.54" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="MOSI" x="7.62" y="5.08" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="MISO" x="7.62" y="7.62" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="SCLK" x="7.62" y="10.16" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="BL" x="7.62" y="12.7" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="MA03-2">
<wire x1="8.89" y1="-5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="8.89" y1="-5.08" x2="8.89" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="8.89" y2="5.08" width="0.4064" layer="94"/>
<wire x1="6.35" y1="2.54" x2="7.62" y2="2.54" width="0.6096" layer="94"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.6096" layer="94"/>
<wire x1="6.35" y1="-2.54" x2="7.62" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<text x="-3.81" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="12.7" y="-2.54" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="12.7" y="0" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="12.7" y="2.54" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="-7.62" y="-2.54" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="4" x="-7.62" y="0" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="6" x="-7.62" y="2.54" visible="pin" length="middle" direction="pas" swaplevel="1"/>
</symbol>
<symbol name="0473460001">
<wire x1="0" y1="7.62" x2="0" y2="5.08" width="0.254" layer="94"/>
<wire x1="0" y1="5.08" x2="0" y2="2.54" width="0.254" layer="94"/>
<wire x1="0" y1="2.54" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="-5.08" width="0.254" layer="94"/>
<wire x1="0" y1="-5.08" x2="0" y2="-7.62" width="0.254" layer="94"/>
<wire x1="0" y1="-7.62" x2="2.54" y2="-10.16" width="0.254" layer="94"/>
<wire x1="2.54" y1="-10.16" x2="7.62" y2="-10.16" width="0.254" layer="94"/>
<wire x1="7.62" y1="-10.16" x2="7.62" y2="10.16" width="0.254" layer="94"/>
<wire x1="7.62" y1="10.16" x2="2.54" y2="10.16" width="0.254" layer="94"/>
<wire x1="2.54" y1="10.16" x2="0" y2="7.62" width="0.254" layer="94"/>
<wire x1="0" y1="5.08" x2="2.54" y2="5.08" width="0.762" layer="94"/>
<wire x1="0" y1="2.54" x2="2.54" y2="2.54" width="0.762" layer="94"/>
<wire x1="0" y1="0" x2="2.54" y2="0" width="0.762" layer="94"/>
<wire x1="0" y1="-2.54" x2="2.54" y2="-2.54" width="0.762" layer="94"/>
<wire x1="0" y1="-5.08" x2="2.54" y2="-5.08" width="0.762" layer="94"/>
<wire x1="2.54" y1="5.08" x2="2.54" y2="7.62" width="0.254" layer="94"/>
<wire x1="2.54" y1="7.62" x2="6.35" y2="7.62" width="0.254" layer="94"/>
<wire x1="6.35" y1="7.62" x2="6.35" y2="-7.62" width="0.254" layer="94"/>
<wire x1="6.35" y1="-7.62" x2="2.54" y2="-7.62" width="0.254" layer="94"/>
<wire x1="2.54" y1="-7.62" x2="2.54" y2="5.08" width="0.254" layer="94"/>
<text x="2.5411" y="10.1644" size="1.27055" layer="95">&gt;NAME</text>
<text x="2.54385" y="-11.4473" size="1.27193125" layer="96">&gt;VALUE</text>
<pin name="VBUS" x="-5.08" y="5.08" length="middle" direction="pwr"/>
<pin name="D-" x="-5.08" y="2.54" length="middle"/>
<pin name="D+" x="-5.08" y="0" length="middle"/>
<pin name="ID" x="-5.08" y="-2.54" length="middle"/>
<pin name="GND" x="-5.08" y="-5.08" length="middle" direction="pas"/>
</symbol>
<symbol name="TLV75533PDBVR">
<wire x1="-7.62" y1="7.62" x2="7.62" y2="7.62" width="0.4064" layer="94"/>
<wire x1="7.62" y1="7.62" x2="7.62" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-7.62" x2="-7.62" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="-7.62" x2="-7.62" y2="7.62" width="0.4064" layer="94"/>
<text x="-7.62218125" y="8.25736875" size="1.778509375" layer="95">&gt;NAME</text>
<text x="-7.635190625" y="-10.1803" size="1.78155" layer="96">&gt;VALUE</text>
<pin name="IN" x="-12.7" y="2.54" length="middle" direction="in"/>
<pin name="EN" x="-12.7" y="0" length="middle" direction="in"/>
<pin name="OUT" x="12.7" y="2.54" length="middle" direction="out" rot="R180"/>
<pin name="GND" x="12.7" y="-5.08" length="middle" direction="pwr" rot="R180"/>
</symbol>
<symbol name="R-US">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.2032" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.2032" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.2032" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
<symbol name="USB-A">
<wire x1="0" y1="7.62" x2="0" y2="-5.08" width="0.254" layer="94"/>
<wire x1="0" y1="-5.08" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="0" y1="7.62" x2="5.08" y2="7.62" width="0.254" layer="94"/>
<text x="0" y="8.89" size="1.778" layer="95">&gt;NAME</text>
<text x="0" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="5.08" y="-2.54" size="2.54" layer="94" rot="R90">USB</text>
<pin name="1" x="-2.54" y="5.08" visible="pad" length="short" direction="pas"/>
<pin name="2" x="-2.54" y="2.54" visible="pad" length="short" direction="pas"/>
<pin name="3" x="-2.54" y="0" visible="pad" length="short" direction="pas"/>
<pin name="4" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas"/>
</symbol>
<symbol name="C-US">
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="-1.016" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-1" x2="2.4892" y2="-1.8542" width="0.254" layer="94" curve="-37.878202"/>
<wire x1="-2.4668" y1="-1.8504" x2="0" y2="-1.0161" width="0.254" layer="94" curve="-37.373024"/>
<text x="1.016" y="0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="1.016" y="-4.191" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="0" y="2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="MA07-2">
<wire x1="3.81" y1="-10.16" x2="-3.81" y2="-10.16" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="2.54" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-7.62" x2="2.54" y2="-7.62" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="-1.27" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="-1.27" y2="-7.62" width="0.6096" layer="94"/>
<wire x1="-3.81" y1="10.16" x2="-3.81" y2="-10.16" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-10.16" x2="3.81" y2="10.16" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="10.16" x2="3.81" y2="10.16" width="0.4064" layer="94"/>
<wire x1="1.27" y1="5.08" x2="2.54" y2="5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-1.27" y2="5.08" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="7.62" x2="-1.27" y2="7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="7.62" x2="2.54" y2="7.62" width="0.6096" layer="94"/>
<text x="-3.81" y="-12.7" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="10.922" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-7.62" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="-5.08" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="-2.54" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="-7.62" y="-7.62" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="4" x="-7.62" y="-5.08" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="6" x="-7.62" y="-2.54" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="7" x="7.62" y="0" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="9" x="7.62" y="2.54" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="VCC" x="7.62" y="5.08" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="8" x="-7.62" y="0" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="10" x="-7.62" y="2.54" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="12" x="-7.62" y="5.08" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="14" x="-7.62" y="7.62" visible="pin" length="middle" direction="pas" swaplevel="1"/>
<pin name="GND" x="7.62" y="7.62" visible="pin" length="middle" direction="pas" swaplevel="1" rot="R180"/>
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
<deviceset name="LCDHEADERS" prefix="SV" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="LCD" symbol="MA10-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA10-1">
<connects>
<connect gate="LCD" pin="BL" pad="10"/>
<connect gate="LCD" pin="CS" pad="6"/>
<connect gate="LCD" pin="D/C" pad="3"/>
<connect gate="LCD" pin="GND" pad="1"/>
<connect gate="LCD" pin="MISO" pad="8"/>
<connect gate="LCD" pin="MOSI" pad="7"/>
<connect gate="LCD" pin="RST" pad="4"/>
<connect gate="LCD" pin="SCLK" pad="9"/>
<connect gate="LCD" pin="SDCS" pad="5"/>
<connect gate="LCD" pin="VIN" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8346/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="DEBUGGER" prefix="SV" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="MA03-2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA03-2">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8334/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="USBMIRCOFEMALE" prefix="P">
<description>Conn Micro USB Type B RCP 5 POS 0.65mm Solder RA SMD 5 Terminal 1 Port T/R</description>
<gates>
<gate name="G$1" symbol="0473460001" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MOLEX_0473460001">
<connects>
<connect gate="G$1" pin="D+" pad="3"/>
<connect gate="G$1" pin="D-" pad="2"/>
<connect gate="G$1" pin="GND" pad="5"/>
<connect gate="G$1" pin="ID" pad="4"/>
<connect gate="G$1" pin="VBUS" pad="1"/>
</connects>
<technologies>
<technology name="">
<attribute name="AVAILABILITY" value="Unavailable"/>
<attribute name="DESCRIPTION" value=" Conn Micro USB Type B RCP 5 POS 0.65mm Solder RA SMD 5 Terminal 1 Port T/R "/>
<attribute name="MF" value="Weidmuller"/>
<attribute name="MP" value="0473460001"/>
<attribute name="PACKAGE" value="None"/>
<attribute name="PRICE" value="None"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="3.3V-REGULATOR" prefix="U">
<description>500mA Low-IQ Small-Size Low-Dropout (LDO) Regulator 5-SOT-23 -40 to 125</description>
<gates>
<gate name="G$1" symbol="TLV75533PDBVR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT95P280X145-5N">
<connects>
<connect gate="G$1" pin="EN" pad="3"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="IN" pad="1"/>
<connect gate="G$1" pin="OUT" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="AVAILABILITY" value="Unavailable"/>
<attribute name="DESCRIPTION" value=" 500mA Low-IQ Small-Size Low-Dropout _LDO_ Regulator 5-SOT-23 -40 to 125 "/>
<attribute name="MF" value="Texas Instruments"/>
<attribute name="MP" value="TLV75533PDBVR"/>
<attribute name="PACKAGE" value="SOT-23 Texas Instruments"/>
<attribute name="PRICE" value="None"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RESISTOR">
<gates>
<gate name="G$1" symbol="R-US" x="0" y="0"/>
</gates>
<devices>
<device name="" package="M1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="USBCON" prefix="X" uservalue="yes">
<description>&lt;b&gt;BERG&lt;/b&gt; USB connector</description>
<gates>
<gate name="G$1" symbol="USB-A" x="-2.54" y="0"/>
</gates>
<devices>
<device name="" package="PN87520">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:6146/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CAP">
<gates>
<gate name="G$1" symbol="C-US" x="0" y="0"/>
</gates>
<devices>
<device name="" package="C1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MISCHEADER" prefix="SV" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="MA07-2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA07-2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="12" pad="12"/>
<connect gate="G$1" pin="14" pad="14"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
<connect gate="G$1" pin="GND" pad="13"/>
<connect gate="G$1" pin="VCC" pad="11"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8330/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="display-hp" urn="urn:adsk.eagle:library:212">
<description>&lt;b&gt;Hewlett Packard LED Displays&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="HLMP6" urn="urn:adsk.eagle:footprint:12942/1" library_version="2">
<description>&lt;b&gt;LED&lt;/b&gt;&lt;p&gt;
2 mm horizontal, RM 2.54 mm</description>
<wire x1="-2.413" y1="-1.27" x2="2.921" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.27" x2="2.921" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.27" x2="-2.413" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.27" x2="-2.413" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="-0.889" x2="-2.794" y2="0.889" width="0.1524" layer="21" curve="-180"/>
<wire x1="-2.794" y1="0.889" x2="-2.413" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="0.889" x2="-2.413" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="-0.889" x2="-2.413" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-0.889" x2="-2.413" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="0.635" x2="-2.794" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-3.048" y1="0.508" x2="-3.048" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="0.254" x2="-3.302" y2="-0.254" width="0.1524" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="3.429" y="0.127" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.429" y="-1.397" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="HLMP6" urn="urn:adsk.eagle:package:12992/1" type="box" library_version="2">
<description>LED
2 mm horizontal, RM 2.54 mm</description>
<packageinstances>
<packageinstance name="HLMP6"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="LED" urn="urn:adsk.eagle:symbol:12940/2" library_version="2">
<wire x1="1.27" y1="2.54" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="2.54" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="2.54" x2="0" y2="2.54" width="0.254" layer="94"/>
<wire x1="0" y1="5.08" x2="0" y2="2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="2.54" x2="-1.27" y2="2.54" width="0.254" layer="94"/>
<wire x1="-2.032" y1="1.778" x2="-3.429" y2="0.381" width="0.1524" layer="94"/>
<wire x1="-3.429" y1="0.381" x2="-2.54" y2="0.762" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="0.762" x2="-3.048" y2="1.27" width="0.1524" layer="94"/>
<wire x1="-3.048" y1="1.27" x2="-3.429" y2="0.381" width="0.1524" layer="94"/>
<wire x1="-3.302" y1="-0.762" x2="-2.413" y2="-0.381" width="0.1524" layer="94"/>
<wire x1="-2.413" y1="-0.381" x2="-2.921" y2="0.127" width="0.1524" layer="94"/>
<wire x1="-2.921" y1="0.127" x2="-3.302" y2="-0.762" width="0.1524" layer="94"/>
<wire x1="-1.905" y1="0.635" x2="-3.302" y2="-0.762" width="0.1524" layer="94"/>
<text x="3.302" y="-1.524" size="1.778" layer="95" rot="R90">&gt;NAME</text>
<text x="5.461" y="-1.524" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="K" x="0" y="-2.54" visible="off" length="short" direction="pas" rot="R90"/>
<pin name="A" x="0" y="5.08" visible="off" length="point" direction="pas" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="HLMP6" urn="urn:adsk.eagle:component:13019/2" prefix="D" library_version="2">
<description>&lt;b&gt;LED&lt;/b&gt;&lt;p&gt;
2mm horizontal</description>
<gates>
<gate name="A" symbol="LED" x="0" y="0"/>
</gates>
<devices>
<device name="" package="HLMP6">
<connects>
<connect gate="A" pin="A" pad="A"/>
<connect gate="A" pin="K" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:12992/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="AGILENT TECHNOLOGIES" constant="no"/>
<attribute name="MPN" value="HLMP-6500" constant="no"/>
<attribute name="OC_FARNELL" value="1652490" constant="no"/>
<attribute name="OC_NEWARK" value="40K0088" constant="no"/>
</technology>
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
<part name="SV2" library="third_eye" deviceset="LCDHEADERS" device="" package3d_urn="urn:adsk.eagle:package:8346/1"/>
<part name="SWD" library="third_eye" deviceset="DEBUGGER" device="" package3d_urn="urn:adsk.eagle:package:8334/1"/>
<part name="P1" library="third_eye" deviceset="USBMIRCOFEMALE" device=""/>
<part name="U2" library="third_eye" deviceset="3.3V-REGULATOR" device=""/>
<part name="D1" library="display-hp" library_urn="urn:adsk.eagle:library:212" deviceset="HLMP6" device="" package3d_urn="urn:adsk.eagle:package:12992/1"/>
<part name="U$2" library="third_eye" deviceset="RESISTOR" device="" value="1K"/>
<part name="X1" library="third_eye" deviceset="USBCON" device="" package3d_urn="urn:adsk.eagle:package:6146/1"/>
<part name="U$3" library="third_eye" deviceset="CAP" device=""/>
<part name="U$4" library="third_eye" deviceset="CAP" device=""/>
<part name="U$5" library="third_eye" deviceset="CAP" device=""/>
<part name="U$6" library="third_eye" deviceset="CAP" device=""/>
<part name="U$7" library="third_eye" deviceset="CAP" device=""/>
<part name="U$8" library="third_eye" deviceset="CAP" device=""/>
<part name="U$9" library="third_eye" deviceset="CAP" device=""/>
<part name="SV1" library="third_eye" deviceset="MISCHEADER" device="" package3d_urn="urn:adsk.eagle:package:8330/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="77.47" y="53.34" smashed="yes"/>
<instance part="SV2" gate="LCD" x="142.24" y="63.5" smashed="yes" rot="R180">
<attribute name="VALUE" x="143.51" y="78.74" size="1.778" layer="96" rot="R180"/>
<attribute name="NAME" x="143.51" y="47.498" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="SWD" gate="1" x="127" y="95.25" smashed="yes">
<attribute name="VALUE" x="123.19" y="87.63" size="1.778" layer="96"/>
<attribute name="NAME" x="123.19" y="101.092" size="1.778" layer="95"/>
</instance>
<instance part="P1" gate="G$1" x="-25.4" y="78.74" smashed="yes" rot="R180">
<attribute name="NAME" x="-27.9411" y="68.5756" size="1.27055" layer="95" rot="R180"/>
<attribute name="VALUE" x="-27.94385" y="90.1873" size="1.27193125" layer="96" rot="R180"/>
</instance>
<instance part="U2" gate="G$1" x="0" y="71.12" smashed="yes">
<attribute name="NAME" x="-7.62218125" y="79.37736875" size="1.778509375" layer="95"/>
<attribute name="VALUE" x="-7.635190625" y="60.9397" size="1.78155" layer="96"/>
</instance>
<instance part="D1" gate="A" x="132.08" y="7.62" smashed="yes">
<attribute name="NAME" x="135.382" y="6.096" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="137.541" y="6.096" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="U$2" gate="G$1" x="132.08" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="130.5814" y="19.05" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="135.382" y="19.05" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="X1" gate="G$1" x="-27.94" y="53.34" smashed="yes" rot="MR0">
<attribute name="NAME" x="-27.94" y="62.23" size="1.778" layer="95" rot="MR0"/>
<attribute name="VALUE" x="-27.94" y="45.72" size="1.778" layer="96" rot="MR0"/>
</instance>
<instance part="U$3" gate="G$1" x="-20.32" y="55.88" smashed="yes">
<attribute name="NAME" x="-19.304" y="56.515" size="1.778" layer="95"/>
<attribute name="VALUE" x="-19.304" y="51.689" size="1.778" layer="96"/>
</instance>
<instance part="U$4" gate="G$1" x="-15.24" y="78.74" smashed="yes">
<attribute name="NAME" x="-14.224" y="79.375" size="1.778" layer="95"/>
<attribute name="VALUE" x="-14.224" y="74.549" size="1.778" layer="96"/>
</instance>
<instance part="U$5" gate="G$1" x="121.92" y="71.12" smashed="yes">
<attribute name="NAME" x="122.936" y="71.755" size="1.778" layer="95"/>
<attribute name="VALUE" x="122.936" y="66.929" size="1.778" layer="96"/>
</instance>
<instance part="U$6" gate="G$1" x="27.94" y="55.88" smashed="yes">
<attribute name="NAME" x="28.956" y="56.515" size="1.778" layer="95"/>
<attribute name="VALUE" x="28.956" y="51.689" size="1.778" layer="96"/>
</instance>
<instance part="U$7" gate="G$1" x="66.04" y="0" smashed="yes">
<attribute name="NAME" x="67.056" y="0.635" size="1.778" layer="95"/>
<attribute name="VALUE" x="67.056" y="-4.191" size="1.778" layer="96"/>
</instance>
<instance part="U$8" gate="G$1" x="106.68" y="71.12" smashed="yes" rot="R90">
<attribute name="NAME" x="106.045" y="72.136" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="110.871" y="72.136" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="U$9" gate="G$1" x="15.24" y="78.74" smashed="yes">
<attribute name="NAME" x="16.256" y="79.375" size="1.778" layer="95"/>
<attribute name="VALUE" x="16.256" y="74.549" size="1.778" layer="96"/>
</instance>
<instance part="SV1" gate="G$1" x="-2.54" y="33.02" smashed="yes">
<attribute name="VALUE" x="-6.35" y="20.32" size="1.778" layer="96"/>
<attribute name="NAME" x="-6.35" y="43.942" size="1.778" layer="95"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<wire x1="33.02" y1="101.6" x2="101.6" y2="101.6" width="0.1524" layer="91"/>
<label x="69.85" y="102.87" size="1.778" layer="95"/>
<pinref part="P1" gate="G$1" pin="GND"/>
<wire x1="-20.32" y1="83.82" x2="-15.24" y2="83.82" width="0.1524" layer="91"/>
<wire x1="-15.24" y1="83.82" x2="15.24" y2="83.82" width="0.1524" layer="91"/>
<wire x1="15.24" y1="83.82" x2="33.02" y2="83.82" width="0.1524" layer="91"/>
<wire x1="33.02" y1="83.82" x2="33.02" y2="71.12" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P1-GND"/>
<wire x1="41.91" y1="71.12" x2="33.02" y2="71.12" width="0.1524" layer="91"/>
<wire x1="33.02" y1="83.82" x2="33.02" y2="101.6" width="0.1524" layer="91"/>
<junction x="33.02" y="83.82"/>
<pinref part="U2" gate="G$1" pin="GND"/>
<wire x1="12.7" y1="66.04" x2="15.24" y2="66.04" width="0.1524" layer="91"/>
<wire x1="15.24" y1="66.04" x2="27.94" y2="66.04" width="0.1524" layer="91"/>
<wire x1="27.94" y1="66.04" x2="33.02" y2="66.04" width="0.1524" layer="91"/>
<wire x1="33.02" y1="66.04" x2="33.02" y2="83.82" width="0.1524" layer="91"/>
<junction x="33.02" y="83.82"/>
<wire x1="15.24" y1="66.04" x2="15.24" y2="50.8" width="0.1524" layer="91"/>
<junction x="15.24" y="66.04"/>
<pinref part="U$1" gate="G$1" pin="P39-GND"/>
<pinref part="SV2" gate="LCD" pin="GND"/>
<wire x1="134.62" y1="73.66" x2="121.92" y2="73.66" width="0.1524" layer="91"/>
<wire x1="121.92" y1="73.66" x2="115.57" y2="73.66" width="0.1524" layer="91"/>
<wire x1="115.57" y1="73.66" x2="101.6" y2="73.66" width="0.1524" layer="91"/>
<wire x1="101.6" y1="73.66" x2="87.63" y2="73.66" width="0.1524" layer="91"/>
<junction x="101.6" y="73.66"/>
<wire x1="101.6" y1="2.54" x2="101.6" y2="71.12" width="0.1524" layer="91"/>
<wire x1="101.6" y1="71.12" x2="101.6" y2="73.66" width="0.1524" layer="91"/>
<wire x1="80.01" y1="2.54" x2="101.6" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P24-GND"/>
<wire x1="80.01" y1="7.62" x2="80.01" y2="2.54" width="0.1524" layer="91"/>
<junction x="80.01" y="2.54"/>
<pinref part="U$1" gate="G$1" pin="P12-GND"/>
<wire x1="49.53" y1="7.62" x2="49.53" y2="2.54" width="0.1524" layer="91"/>
<wire x1="49.53" y1="2.54" x2="66.04" y2="2.54" width="0.1524" layer="91"/>
<wire x1="66.04" y1="2.54" x2="80.01" y2="2.54" width="0.1524" layer="91"/>
<wire x1="101.6" y1="101.6" x2="101.6" y2="73.66" width="0.1524" layer="91"/>
<junction x="101.6" y="73.66"/>
<pinref part="SWD" gate="1" pin="2"/>
<wire x1="119.38" y1="92.71" x2="115.57" y2="92.71" width="0.1524" layer="91"/>
<wire x1="115.57" y1="92.71" x2="115.57" y2="73.66" width="0.1524" layer="91"/>
<junction x="115.57" y="73.66"/>
<pinref part="X1" gate="G$1" pin="4"/>
<wire x1="-25.4" y1="50.8" x2="-20.32" y2="50.8" width="0.1524" layer="91"/>
<junction x="15.24" y="50.8"/>
<wire x1="-20.32" y1="50.8" x2="15.24" y2="50.8" width="0.1524" layer="91"/>
<wire x1="2.54" y1="40.64" x2="5.08" y2="40.64" width="0.1524" layer="91"/>
<wire x1="5.08" y1="40.64" x2="15.24" y2="40.64" width="0.1524" layer="91"/>
<wire x1="15.24" y1="40.64" x2="15.24" y2="50.8" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="2"/>
<junction x="-20.32" y="50.8"/>
<pinref part="U$4" gate="G$1" pin="1"/>
<wire x1="-15.24" y1="81.28" x2="-15.24" y2="83.82" width="0.1524" layer="91"/>
<junction x="-15.24" y="83.82"/>
<pinref part="U$5" gate="G$1" pin="1"/>
<junction x="121.92" y="73.66"/>
<pinref part="U$7" gate="G$1" pin="1"/>
<junction x="66.04" y="2.54"/>
<pinref part="U$6" gate="G$1" pin="1"/>
<wire x1="27.94" y1="58.42" x2="27.94" y2="66.04" width="0.1524" layer="91"/>
<junction x="27.94" y="66.04"/>
<pinref part="U$8" gate="G$1" pin="1"/>
<wire x1="104.14" y1="71.12" x2="101.6" y2="71.12" width="0.1524" layer="91"/>
<junction x="101.6" y="71.12"/>
<pinref part="U$9" gate="G$1" pin="1"/>
<wire x1="15.24" y1="81.28" x2="15.24" y2="83.82" width="0.1524" layer="91"/>
<junction x="15.24" y="83.82"/>
<label x="5.08" y="40.64" size="1.778" layer="95"/>
<pinref part="SV1" gate="G$1" pin="GND"/>
<junction x="5.08" y="40.64"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="SV2" gate="LCD" pin="MOSI"/>
<pinref part="U$1" gate="G$1" pin="P33-P0.19"/>
<wire x1="134.62" y1="58.42" x2="87.63" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P32-P0.18"/>
<wire x1="87.63" y1="55.88" x2="127" y2="55.88" width="0.1524" layer="91"/>
<wire x1="127" y1="55.88" x2="127" y2="53.34" width="0.1524" layer="91"/>
<pinref part="SV2" gate="LCD" pin="SCLK"/>
<wire x1="127" y1="53.34" x2="134.62" y2="53.34" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P31-P0.17"/>
<wire x1="87.63" y1="53.34" x2="125.73" y2="53.34" width="0.1524" layer="91"/>
<wire x1="125.73" y1="53.34" x2="125.73" y2="50.8" width="0.1524" layer="91"/>
<pinref part="SV2" gate="LCD" pin="BL"/>
<wire x1="125.73" y1="50.8" x2="134.62" y2="50.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="SWD" gate="1" pin="5"/>
<wire x1="139.7" y1="97.79" x2="144.78" y2="97.79" width="0.1524" layer="91"/>
<wire x1="144.78" y1="97.79" x2="144.78" y2="80.01" width="0.1524" layer="91"/>
<wire x1="144.78" y1="80.01" x2="93.98" y2="80.01" width="0.1524" layer="91"/>
<wire x1="93.98" y1="80.01" x2="93.98" y2="68.58" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P37-SWDIO"/>
<wire x1="93.98" y1="68.58" x2="87.63" y2="68.58" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P36-SWDCLK"/>
<wire x1="87.63" y1="66.04" x2="87.63" y2="67.31" width="0.1524" layer="91"/>
<wire x1="87.63" y1="67.31" x2="95.25" y2="67.31" width="0.1524" layer="91"/>
<wire x1="95.25" y1="67.31" x2="95.25" y2="78.74" width="0.1524" layer="91"/>
<wire x1="95.25" y1="78.74" x2="143.51" y2="78.74" width="0.1524" layer="91"/>
<wire x1="143.51" y1="78.74" x2="143.51" y2="95.25" width="0.1524" layer="91"/>
<pinref part="SWD" gate="1" pin="3"/>
<wire x1="143.51" y1="95.25" x2="139.7" y2="95.25" width="0.1524" layer="91"/>
</segment>
</net>
<net name="5V" class="0">
<segment>
<pinref part="P1" gate="G$1" pin="VBUS"/>
<wire x1="-20.32" y1="73.66" x2="-17.78" y2="73.66" width="0.1524" layer="91"/>
<pinref part="U2" gate="G$1" pin="IN"/>
<wire x1="-17.78" y1="73.66" x2="-15.24" y2="73.66" width="0.1524" layer="91"/>
<wire x1="-15.24" y1="73.66" x2="-12.7" y2="73.66" width="0.1524" layer="91"/>
<wire x1="-17.78" y1="73.66" x2="-17.78" y2="58.42" width="0.1524" layer="91"/>
<junction x="-17.78" y="73.66"/>
<pinref part="X1" gate="G$1" pin="1"/>
<wire x1="-17.78" y1="58.42" x2="-20.32" y2="58.42" width="0.1524" layer="91"/>
<pinref part="U2" gate="G$1" pin="EN"/>
<wire x1="-20.32" y1="58.42" x2="-25.4" y2="58.42" width="0.1524" layer="91"/>
<wire x1="-12.7" y1="71.12" x2="-12.7" y2="73.66" width="0.1524" layer="91"/>
<junction x="-12.7" y="73.66"/>
<pinref part="U$3" gate="G$1" pin="1"/>
<junction x="-20.32" y="58.42"/>
<pinref part="U$4" gate="G$1" pin="2"/>
<junction x="-15.24" y="73.66"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="1"/>
<pinref part="D1" gate="A" pin="A"/>
<wire x1="132.08" y1="17.78" x2="132.08" y2="12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="3.3V" class="0">
<segment>
<pinref part="U2" gate="G$1" pin="OUT"/>
<wire x1="20.32" y1="73.66" x2="15.24" y2="73.66" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P11-VDD"/>
<wire x1="15.24" y1="73.66" x2="12.7" y2="73.66" width="0.1524" layer="91"/>
<wire x1="41.91" y1="38.1" x2="27.94" y2="38.1" width="0.1524" layer="91"/>
<junction x="20.32" y="38.1"/>
<wire x1="20.32" y1="38.1" x2="20.32" y2="73.66" width="0.1524" layer="91"/>
<label x="22.86" y="38.1" size="1.778" layer="95"/>
<wire x1="20.32" y1="38.1" x2="20.32" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="20.32" y1="38.1" x2="27.94" y2="38.1" width="0.1524" layer="91"/>
<wire x1="20.32" y1="-5.08" x2="66.04" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="2"/>
<wire x1="66.04" y1="-5.08" x2="111.76" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="132.08" y1="27.94" x2="111.76" y2="27.94" width="0.1524" layer="91"/>
<wire x1="111.76" y1="27.94" x2="111.76" y2="5.08" width="0.1524" layer="91"/>
<wire x1="111.76" y1="5.08" x2="111.76" y2="-5.08" width="0.1524" layer="91"/>
<junction x="111.76" y="27.94"/>
<pinref part="D1" gate="A" pin="K"/>
<wire x1="132.08" y1="5.08" x2="111.76" y2="5.08" width="0.1524" layer="91"/>
<junction x="111.76" y="5.08"/>
<wire x1="111.76" y1="27.94" x2="111.76" y2="66.04" width="0.1524" layer="91"/>
<pinref part="SWD" gate="1" pin="6"/>
<wire x1="111.76" y1="66.04" x2="111.76" y2="71.12" width="0.1524" layer="91"/>
<wire x1="111.76" y1="71.12" x2="111.76" y2="97.79" width="0.1524" layer="91"/>
<wire x1="119.38" y1="97.79" x2="111.76" y2="97.79" width="0.1524" layer="91"/>
<wire x1="2.54" y1="38.1" x2="5.08" y2="38.1" width="0.1524" layer="91"/>
<pinref part="SV2" gate="LCD" pin="VIN"/>
<wire x1="5.08" y1="38.1" x2="20.32" y2="38.1" width="0.1524" layer="91"/>
<wire x1="134.62" y1="71.12" x2="129.54" y2="71.12" width="0.1524" layer="91"/>
<wire x1="129.54" y1="71.12" x2="129.54" y2="66.04" width="0.1524" layer="91"/>
<wire x1="129.54" y1="66.04" x2="121.92" y2="66.04" width="0.1524" layer="91"/>
<junction x="111.76" y="66.04"/>
<pinref part="U$5" gate="G$1" pin="2"/>
<wire x1="121.92" y1="66.04" x2="111.76" y2="66.04" width="0.1524" layer="91"/>
<junction x="121.92" y="66.04"/>
<pinref part="U$7" gate="G$1" pin="2"/>
<junction x="66.04" y="-5.08"/>
<pinref part="U$6" gate="G$1" pin="2"/>
<wire x1="27.94" y1="50.8" x2="27.94" y2="38.1" width="0.1524" layer="91"/>
<junction x="27.94" y="38.1"/>
<pinref part="U$8" gate="G$1" pin="2"/>
<junction x="111.76" y="71.12"/>
<pinref part="U$9" gate="G$1" pin="2"/>
<junction x="15.24" y="73.66"/>
<label x="5.08" y="38.1" size="1.778" layer="95"/>
<pinref part="SV1" gate="G$1" pin="VCC"/>
<junction x="5.08" y="38.1"/>
</segment>
</net>
<net name="LEDR" class="0">
<segment>
<wire x1="2.54" y1="35.56" x2="5.08" y2="35.56" width="0.1524" layer="91"/>
<wire x1="5.08" y1="35.56" x2="33.02" y2="35.56" width="0.1524" layer="91"/>
<wire x1="33.02" y1="35.56" x2="33.02" y2="60.96" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P2-P0.25"/>
<wire x1="33.02" y1="60.96" x2="41.91" y2="60.96" width="0.1524" layer="91"/>
<label x="5.08" y="35.56" size="1.778" layer="95"/>
<pinref part="SV1" gate="G$1" pin="9"/>
<junction x="5.08" y="35.56"/>
</segment>
</net>
<net name="LEDL" class="0">
<segment>
<wire x1="2.54" y1="33.02" x2="5.08" y2="33.02" width="0.1524" layer="91"/>
<wire x1="5.08" y1="33.02" x2="35.56" y2="33.02" width="0.1524" layer="91"/>
<wire x1="35.56" y1="33.02" x2="35.56" y2="58.42" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P3-P0.26"/>
<wire x1="35.56" y1="58.42" x2="41.91" y2="58.42" width="0.1524" layer="91"/>
<label x="5.08" y="33.02" size="1.778" layer="95"/>
<pinref part="SV1" gate="G$1" pin="7"/>
<junction x="5.08" y="33.02"/>
</segment>
</net>
<net name="SPEAKER" class="0">
<segment>
<wire x1="2.54" y1="30.48" x2="5.08" y2="30.48" width="0.1524" layer="91"/>
<wire x1="5.08" y1="30.48" x2="38.1" y2="30.48" width="0.1524" layer="91"/>
<wire x1="38.1" y1="30.48" x2="38.1" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P4-P0.27"/>
<wire x1="38.1" y1="55.88" x2="41.91" y2="55.88" width="0.1524" layer="91"/>
<label x="5.08" y="30.48" size="1.778" layer="95"/>
<pinref part="SV1" gate="G$1" pin="5"/>
<junction x="5.08" y="30.48"/>
</segment>
</net>
<net name="POT" class="0">
<segment>
<wire x1="2.54" y1="27.94" x2="5.08" y2="27.94" width="0.1524" layer="91"/>
<wire x1="5.08" y1="27.94" x2="40.64" y2="27.94" width="0.1524" layer="91"/>
<wire x1="40.64" y1="27.94" x2="40.64" y2="53.34" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P5-P0.28/AIN4"/>
<wire x1="40.64" y1="53.34" x2="41.91" y2="53.34" width="0.1524" layer="91"/>
<label x="5.08" y="27.94" size="1.778" layer="95"/>
<pinref part="SV1" gate="G$1" pin="3"/>
<junction x="5.08" y="27.94"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
