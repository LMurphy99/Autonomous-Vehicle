<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.1">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="10" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
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
<layer number="51" name="tDocu" color="26" fill="1" visible="no" active="no"/>
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
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-molex" urn="urn:adsk.eagle:library:165">
<description>&lt;b&gt;Molex Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="7395-07" urn="urn:adsk.eagle:footprint:8078280/1" library_version="5">
<description>&lt;b&gt;KK® 254 Wire-to-Board Header, Right Angle, with Friction Lock, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022057078_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="8.89" y1="-5.08" x2="8.89" y2="-1.905" width="0.254" layer="21"/>
<wire x1="8.89" y1="-1.905" x2="8.255" y2="-1.905" width="0.254" layer="21"/>
<wire x1="8.255" y1="-1.905" x2="8.255" y2="-2.54" width="0.254" layer="21"/>
<wire x1="8.255" y1="-2.54" x2="6.985" y2="-2.54" width="0.254" layer="21"/>
<wire x1="6.985" y1="-2.54" x2="6.985" y2="-1.905" width="0.254" layer="21"/>
<wire x1="6.985" y1="-1.905" x2="5.715" y2="-1.905" width="0.254" layer="21"/>
<wire x1="5.715" y1="-1.905" x2="5.715" y2="-2.54" width="0.254" layer="21"/>
<wire x1="5.715" y1="-2.54" x2="4.445" y2="-2.54" width="0.254" layer="21"/>
<wire x1="4.445" y1="-2.54" x2="4.445" y2="-1.905" width="0.254" layer="21"/>
<wire x1="4.445" y1="-1.905" x2="3.175" y2="-1.905" width="0.254" layer="21"/>
<wire x1="3.175" y1="-1.905" x2="3.175" y2="-2.54" width="0.254" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.254" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.905" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-1.905" y1="-1.905" x2="-1.905" y2="-2.54" width="0.254" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.254" layer="21"/>
<wire x1="-3.175" y1="-2.54" x2="-3.175" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-3.175" y1="-1.905" x2="-4.445" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-4.445" y1="-1.905" x2="-4.445" y2="-2.54" width="0.254" layer="21"/>
<wire x1="-4.445" y1="-2.54" x2="-5.715" y2="-2.54" width="0.254" layer="21"/>
<wire x1="-5.715" y1="-2.54" x2="-5.715" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-5.715" y1="-1.905" x2="-6.985" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-6.985" y1="-1.905" x2="-6.985" y2="-2.54" width="0.254" layer="21"/>
<wire x1="-6.985" y1="-2.54" x2="-8.255" y2="-2.54" width="0.254" layer="21"/>
<wire x1="-8.255" y1="-2.54" x2="-8.255" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-8.255" y1="-1.905" x2="-8.89" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-8.89" y1="-1.905" x2="-8.89" y2="-5.08" width="0.254" layer="21"/>
<wire x1="-8.89" y1="-5.08" x2="-8.001" y2="-5.08" width="0.254" layer="21"/>
<wire x1="-8.001" y1="-5.08" x2="-0.381" y2="-5.08" width="0.254" layer="21"/>
<wire x1="-0.381" y1="-5.08" x2="2.159" y2="-5.08" width="0.254" layer="21"/>
<wire x1="2.159" y1="-5.08" x2="2.4892" y2="-5.08" width="0.127" layer="21"/>
<wire x1="2.4892" y1="-5.08" x2="7.62" y2="-5.08" width="0.254" layer="21"/>
<wire x1="7.62" y1="-5.08" x2="8.89" y2="-5.08" width="0.254" layer="21"/>
<wire x1="7.62" y1="-5.08" x2="7.62" y2="-13.081" width="0.254" layer="21"/>
<wire x1="7.62" y1="-13.081" x2="7.366" y2="-13.589" width="0.254" layer="21"/>
<wire x1="7.366" y1="-13.589" x2="2.4892" y2="-13.589" width="0.254" layer="21"/>
<wire x1="2.4892" y1="-13.589" x2="2.4892" y2="-11.6586" width="0.254" layer="21"/>
<wire x1="2.4892" y1="-11.6586" x2="2.4892" y2="-5.08" width="0.254" layer="21"/>
<wire x1="-7.62" y1="-5.08" x2="-7.62" y2="-13.081" width="0.254" layer="21"/>
<wire x1="-7.62" y1="-13.081" x2="-7.366" y2="-13.589" width="0.254" layer="21"/>
<wire x1="-7.366" y1="-13.589" x2="-2.4892" y2="-13.589" width="0.254" layer="21"/>
<wire x1="-2.4892" y1="-13.589" x2="-2.4892" y2="-5.08" width="0.254" layer="21"/>
<wire x1="2.54" y1="-5.08" x2="2.159" y2="-5.08" width="0.254" layer="51"/>
<wire x1="2.159" y1="-5.08" x2="2.159" y2="-11.303" width="0.254" layer="21"/>
<wire x1="2.159" y1="-11.303" x2="2.4892" y2="-11.6586" width="0.254" layer="21"/>
<wire x1="-8.001" y1="-5.08" x2="-8.001" y2="-11.303" width="0.254" layer="21"/>
<wire x1="-8.001" y1="-11.303" x2="-7.6708" y2="-11.6586" width="0.254" layer="21"/>
<wire x1="8.001" y1="-5.08" x2="8.001" y2="-11.303" width="0.254" layer="21"/>
<wire x1="8.001" y1="-11.303" x2="7.6708" y2="-11.6586" width="0.254" layer="21"/>
<wire x1="-2.159" y1="-5.08" x2="-2.159" y2="-11.303" width="0.254" layer="21"/>
<wire x1="-2.159" y1="-11.303" x2="-2.4892" y2="-11.6586" width="0.254" layer="21"/>
<wire x1="1.905" y1="-1.905" x2="0.635" y2="-1.905" width="0.254" layer="21"/>
<wire x1="0.635" y1="-1.905" x2="0.635" y2="-2.54" width="0.254" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.254" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-0.635" y2="-1.905" width="0.254" layer="21"/>
<wire x1="-0.635" y1="-1.905" x2="-1.905" y2="-1.905" width="0.254" layer="21"/>
<wire x1="0.381" y1="-5.08" x2="0.381" y2="-11.303" width="0.254" layer="21"/>
<wire x1="0.381" y1="-11.303" x2="0.0508" y2="-11.6586" width="0.254" layer="21"/>
<wire x1="-0.381" y1="-5.08" x2="-0.381" y2="-11.303" width="0.254" layer="21"/>
<wire x1="-0.381" y1="-11.303" x2="-0.0508" y2="-11.6586" width="0.254" layer="21"/>
<pad name="1" x="7.62" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="-5.08" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="-7.62" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="0" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="8.9901" y="2.9479" size="1.016" layer="25" ratio="10" rot="R180">&gt;NAME</text>
<text x="7.1359" y="-3.4021" size="1.27" layer="21" ratio="14" rot="R270">1</text>
<text x="-8.0533" y="-3.2751" size="1.27" layer="21" ratio="14" rot="R270">7</text>
<text x="-3.7099" y="2.9479" size="0.8128" layer="27" ratio="10" rot="R180">&gt;VALUE</text>
<rectangle x1="7.366" y1="-0.254" x2="7.874" y2="0.254" layer="51"/>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-2.794" y1="-0.254" x2="-2.286" y2="0.254" layer="51"/>
<rectangle x1="-5.334" y1="-0.254" x2="-4.826" y2="0.254" layer="51"/>
<rectangle x1="-7.874" y1="-0.254" x2="-7.366" y2="0.254" layer="51"/>
<rectangle x1="7.366" y1="-2.5146" x2="7.874" y2="-0.2794" layer="51"/>
<rectangle x1="4.826" y1="-2.5146" x2="5.334" y2="-0.2794" layer="51"/>
<rectangle x1="2.286" y1="-2.5146" x2="2.794" y2="-0.2794" layer="51"/>
<rectangle x1="-2.794" y1="-2.5146" x2="-2.286" y2="-0.2794" layer="51"/>
<rectangle x1="-5.334" y1="-2.5146" x2="-4.826" y2="-0.2794" layer="51"/>
<rectangle x1="-7.874" y1="-2.5146" x2="-7.366" y2="-0.2794" layer="51"/>
<rectangle x1="-0.254" y1="-2.5146" x2="0.254" y2="-0.2794" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="6410-07" urn="urn:adsk.eagle:footprint:8078281/1" library_version="5">
<description>&lt;b&gt;KK® 254 Wire-to-Board Header, Vertical, with Friction Lock, 7 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022272071_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-7.62" y1="2.54" x2="-7.62" y2="2.921" width="0.127" layer="21"/>
<wire x1="-8.89" y1="0" x2="-8.89" y2="2.921" width="0.254" layer="21"/>
<wire x1="-8.89" y1="2.921" x2="-7.62" y2="2.921" width="0.254" layer="21"/>
<wire x1="-7.62" y1="2.921" x2="-7.366" y2="2.921" width="0.254" layer="21"/>
<wire x1="-7.366" y1="2.921" x2="-2.54" y2="2.921" width="0.254" layer="21"/>
<wire x1="-2.54" y1="2.921" x2="-1.27" y2="2.921" width="0.254" layer="21"/>
<wire x1="8.89" y1="0" x2="8.89" y2="2.921" width="0.254" layer="21"/>
<wire x1="8.89" y1="2.921" x2="-1.27" y2="2.921" width="0.254" layer="21"/>
<wire x1="-8.89" y1="0" x2="-8.89" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-8.89" y1="-2.921" x2="-1.397" y2="-2.921" width="0.254" layer="21"/>
<wire x1="8.89" y1="0" x2="8.89" y2="-2.921" width="0.254" layer="21"/>
<wire x1="8.89" y1="-2.921" x2="-1.397" y2="-2.921" width="0.127" layer="21"/>
<wire x1="-8.636" y1="-2.921" x2="-8.255" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-8.255" y1="-2.921" x2="-8.255" y2="-2.286" width="0.254" layer="21"/>
<wire x1="-8.255" y1="-2.286" x2="-6.985" y2="-2.286" width="0.254" layer="21"/>
<wire x1="-6.985" y1="-2.286" x2="-6.985" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-6.985" y1="-2.921" x2="-5.715" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-5.715" y1="-2.921" x2="-5.715" y2="-2.286" width="0.254" layer="21"/>
<wire x1="-5.715" y1="-2.286" x2="-4.445" y2="-2.286" width="0.254" layer="21"/>
<wire x1="-4.445" y1="-2.286" x2="-4.445" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-4.445" y1="-2.921" x2="-3.175" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-3.175" y1="-2.921" x2="-3.175" y2="-2.286" width="0.254" layer="21"/>
<wire x1="-3.175" y1="-2.286" x2="-1.905" y2="-2.286" width="0.254" layer="21"/>
<wire x1="-1.905" y1="-2.286" x2="-1.905" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-1.905" y1="-2.921" x2="1.905" y2="-2.921" width="0.254" layer="21"/>
<wire x1="3.175" y1="-2.921" x2="4.445" y2="-2.921" width="0.254" layer="21"/>
<wire x1="4.445" y1="-2.921" x2="4.445" y2="-2.286" width="0.254" layer="21"/>
<wire x1="4.445" y1="-2.286" x2="5.715" y2="-2.286" width="0.254" layer="21"/>
<wire x1="5.715" y1="-2.286" x2="5.715" y2="-2.921" width="0.254" layer="21"/>
<wire x1="5.715" y1="-2.921" x2="6.985" y2="-2.921" width="0.254" layer="21"/>
<wire x1="6.985" y1="-2.921" x2="6.985" y2="-2.286" width="0.254" layer="21"/>
<wire x1="6.985" y1="-2.286" x2="8.255" y2="-2.286" width="0.254" layer="21"/>
<wire x1="8.255" y1="-2.286" x2="8.255" y2="-2.921" width="0.254" layer="21"/>
<wire x1="8.255" y1="-2.921" x2="8.89" y2="-2.921" width="0.254" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="2.921" width="0.254" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.366" y2="1.905" width="0.254" layer="21"/>
<wire x1="-7.366" y1="1.905" x2="-2.54" y2="1.905" width="0.254" layer="21"/>
<wire x1="-2.54" y1="1.905" x2="-2.54" y2="2.921" width="0.254" layer="21"/>
<wire x1="2.54" y1="1.905" x2="2.54" y2="2.921" width="0.254" layer="21"/>
<wire x1="2.54" y1="1.905" x2="7.366" y2="1.905" width="0.254" layer="21"/>
<wire x1="7.366" y1="1.905" x2="7.62" y2="1.905" width="0.254" layer="21"/>
<wire x1="7.62" y1="1.905" x2="7.62" y2="2.921" width="0.254" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.366" y2="1.397" width="0.254" layer="21"/>
<wire x1="-7.366" y1="1.397" x2="-2.794" y2="1.397" width="0.254" layer="21"/>
<wire x1="-2.794" y1="1.397" x2="-2.54" y2="1.905" width="0.254" layer="21"/>
<wire x1="-7.366" y1="2.921" x2="-7.366" y2="1.905" width="0.254" layer="21"/>
<wire x1="2.54" y1="1.905" x2="2.794" y2="1.397" width="0.254" layer="21"/>
<wire x1="2.794" y1="1.397" x2="7.366" y2="1.397" width="0.254" layer="21"/>
<wire x1="7.366" y1="1.397" x2="7.62" y2="1.905" width="0.254" layer="21"/>
<wire x1="7.366" y1="2.921" x2="7.366" y2="1.905" width="0.254" layer="21"/>
<wire x1="-0.635" y1="-2.921" x2="-0.635" y2="-2.286" width="0.254" layer="21"/>
<wire x1="-0.635" y1="-2.286" x2="0.635" y2="-2.286" width="0.254" layer="21"/>
<wire x1="0.635" y1="-2.286" x2="0.635" y2="-2.921" width="0.254" layer="21"/>
<wire x1="1.905" y1="-2.921" x2="1.905" y2="-2.286" width="0.254" layer="21"/>
<wire x1="1.905" y1="-2.286" x2="3.175" y2="-2.286" width="0.254" layer="21"/>
<wire x1="3.175" y1="-2.286" x2="3.175" y2="-2.921" width="0.254" layer="21"/>
<pad name="3" x="2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="1" x="7.62" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="0" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="-7.62" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="-5.08" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-8.8631" y="-4.5989" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="9.3711" y="-0.4841" size="1.27" layer="21" ratio="14">1</text>
<text x="-10.0599" y="-0.5603" size="1.27" layer="21" ratio="14">7</text>
<text x="2.1859" y="-4.5989" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="7.366" y1="-0.254" x2="7.874" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
<rectangle x1="-2.794" y1="-0.254" x2="-2.286" y2="0.254" layer="51"/>
<rectangle x1="-7.874" y1="-0.254" x2="-7.366" y2="0.254" layer="51"/>
<rectangle x1="-5.334" y1="-0.254" x2="-4.826" y2="0.254" layer="51"/>
</package>
<package name="22-23-2081" urn="urn:adsk.eagle:footprint:8078265/1" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 8 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232081_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-10.16" y1="3.175" x2="10.16" y2="3.175" width="0.254" layer="21"/>
<wire x1="10.16" y1="3.175" x2="10.16" y2="1.27" width="0.254" layer="21"/>
<wire x1="10.16" y1="1.27" x2="10.16" y2="-3.175" width="0.254" layer="21"/>
<wire x1="10.16" y1="-3.175" x2="-10.16" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-10.16" y1="-3.175" x2="-10.16" y2="1.27" width="0.254" layer="21"/>
<wire x1="-10.16" y1="1.27" x2="-10.16" y2="3.175" width="0.254" layer="21"/>
<wire x1="-10.16" y1="1.27" x2="10.16" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-8.89" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="-3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="5" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="6" x="3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="7" x="6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="8" x="8.89" y="0" drill="1" shape="long" rot="R90"/>
<text x="-10.16" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.16" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="22-23-2041" urn="urn:adsk.eagle:footprint:8078261/1" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 4 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232041_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-5.08" y1="3.175" x2="5.08" y2="3.175" width="0.254" layer="21"/>
<wire x1="5.08" y1="3.175" x2="5.08" y2="1.27" width="0.254" layer="21"/>
<wire x1="5.08" y1="1.27" x2="5.08" y2="-3.175" width="0.254" layer="21"/>
<wire x1="5.08" y1="-3.175" x2="-5.08" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-5.08" y1="-3.175" x2="-5.08" y2="1.27" width="0.254" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="3.175" width="0.254" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="5.08" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="3.81" y="0" drill="1" shape="long" rot="R90"/>
<text x="-5.08" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="22-23-2051" urn="urn:adsk.eagle:footprint:8078262/1" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 5 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232051_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-6.35" y1="3.175" x2="6.35" y2="3.175" width="0.254" layer="21"/>
<wire x1="6.35" y1="3.175" x2="6.35" y2="1.27" width="0.254" layer="21"/>
<wire x1="6.35" y1="1.27" x2="6.35" y2="-3.175" width="0.254" layer="21"/>
<wire x1="6.35" y1="-3.175" x2="-6.35" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-6.35" y1="-3.175" x2="-6.35" y2="1.27" width="0.254" layer="21"/>
<wire x1="-6.35" y1="1.27" x2="-6.35" y2="3.175" width="0.254" layer="21"/>
<wire x1="-6.35" y1="1.27" x2="6.35" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-2.54" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="0" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="2.54" y="0" drill="1" shape="long" rot="R90"/>
<pad name="5" x="5.08" y="0" drill="1" shape="long" rot="R90"/>
<text x="-6.35" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.35" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="22-23-2021" urn="urn:adsk.eagle:footprint:8078259/1" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 2 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232021_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-2.54" y1="3.175" x2="2.54" y2="3.175" width="0.254" layer="21"/>
<wire x1="2.54" y1="3.175" x2="2.54" y2="1.27" width="0.254" layer="21"/>
<wire x1="2.54" y1="1.27" x2="2.54" y2="-3.175" width="0.254" layer="21"/>
<wire x1="2.54" y1="-3.175" x2="-2.54" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-2.54" y1="-3.175" x2="-2.54" y2="1.27" width="0.254" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="-2.54" y2="3.175" width="0.254" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="2.54" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<text x="-2.54" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="7395-07" urn="urn:adsk.eagle:package:8078654/1" type="box" library_version="5">
<description>&lt;b&gt;KK® 254 Wire-to-Board Header, Right Angle, with Friction Lock, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022057078_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="7395-07"/>
</packageinstances>
</package3d>
<package3d name="6410-07" urn="urn:adsk.eagle:package:8078655/1" type="box" library_version="5">
<description>&lt;b&gt;KK® 254 Wire-to-Board Header, Vertical, with Friction Lock, 7 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022272071_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="6410-07"/>
</packageinstances>
</package3d>
<package3d name="22-23-2081" urn="urn:adsk.eagle:package:8078639/1" type="box" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 8 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232081_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="22-23-2081"/>
</packageinstances>
</package3d>
<package3d name="22-23-2041" urn="urn:adsk.eagle:package:8078635/1" type="box" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 4 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232041_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="22-23-2041"/>
</packageinstances>
</package3d>
<package3d name="22-23-2051" urn="urn:adsk.eagle:package:8078636/1" type="box" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 5 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232051_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="22-23-2051"/>
</packageinstances>
</package3d>
<package3d name="22-23-2021" urn="urn:adsk.eagle:package:8078633/1" type="box" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 2 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232021_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="22-23-2021"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="MV" urn="urn:adsk.eagle:symbol:6783/2" library_version="5">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M" urn="urn:adsk.eagle:symbol:6785/2" library_version="5">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="22-?-07" urn="urn:adsk.eagle:component:8078974/3" prefix="J" library_version="5">
<description>&lt;b&gt;CONNECTOR&lt;/b&gt;&lt;p&gt;
wire to board 2.54 mm (.1 inch) pitch header</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="15.24" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="10.16" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="0" y="-10.16" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="0" y="-15.24" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="05-7078" package="7395-07">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078654/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="27-2071" package="6410-07">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078655/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2081" urn="urn:adsk.eagle:component:8078932/3" prefix="X" library_version="5">
<description>.100" (2.54mm) Center Header - 8 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="7.62" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="0" y="-7.62" addlevel="always" swaplevel="1"/>
<gate name="-8" symbol="M" x="0" y="-10.16" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2081">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078639/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2081" constant="no"/>
<attribute name="OC_FARNELL" value="1756826" constant="no"/>
<attribute name="OC_NEWARK" value="01C7592" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2041" urn="urn:adsk.eagle:component:8078936/3" prefix="X" library_version="5">
<description>.100" (2.54mm) Center Header - 4 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2041">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078635/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2041" constant="no"/>
<attribute name="OC_FARNELL" value="1462920" constant="no"/>
<attribute name="OC_NEWARK" value="38C0355" constant="no"/>
<attribute name="POPULARITY" value="21" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2051" urn="urn:adsk.eagle:component:8078935/3" prefix="X" library_version="5">
<description>.100" (2.54mm) Center Header - 5 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2051">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078636/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2051" constant="no"/>
<attribute name="OC_FARNELL" value="1462952" constant="no"/>
<attribute name="OC_NEWARK" value="38C9178" constant="no"/>
<attribute name="POPULARITY" value="4" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2021" urn="urn:adsk.eagle:component:8078938/3" prefix="X" library_version="5">
<description>.100" (2.54mm) Center Header - 2 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2021">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078633/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2021" constant="no"/>
<attribute name="OC_FARNELL" value="1462926" constant="no"/>
<attribute name="OC_NEWARK" value="25C3832" constant="no"/>
<attribute name="POPULARITY" value="40" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-lsta" urn="urn:adsk.eagle:library:161">
<description>&lt;b&gt;Female Headers etc.&lt;/b&gt;&lt;p&gt;
Naming:&lt;p&gt;
FE = female&lt;p&gt;
# contacts - # rows&lt;p&gt;
W = angled&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="FE15" urn="urn:adsk.eagle:footprint:8165/1" library_version="2">
<description>&lt;b&gt;FEMALE HEADER&lt;/b&gt;</description>
<wire x1="-19.05" y1="1.27" x2="-19.05" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-19.05" y1="-1.27" x2="-16.764" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-16.764" y1="-1.27" x2="-16.51" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-16.51" y1="-1.016" x2="-16.256" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-16.256" y1="-1.27" x2="-14.224" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-14.224" y1="-1.27" x2="-13.97" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-13.97" y1="-1.016" x2="-13.716" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-13.716" y1="-1.27" x2="-11.684" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-11.684" y1="-1.27" x2="-11.43" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="-1.016" x2="-11.176" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-11.176" y1="-1.27" x2="-9.144" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-9.144" y1="-1.27" x2="-8.89" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-1.016" x2="-8.636" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="1.27" x2="-8.89" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="1.016" x2="-9.144" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-9.144" y1="1.27" x2="-11.176" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-11.176" y1="1.27" x2="-11.43" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="1.016" x2="-11.684" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-11.684" y1="1.27" x2="-13.716" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-13.716" y1="1.27" x2="-13.97" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-13.97" y1="1.016" x2="-14.224" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-14.224" y1="1.27" x2="-16.256" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-16.256" y1="1.27" x2="-16.51" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-16.51" y1="1.016" x2="-16.764" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-16.764" y1="1.27" x2="-19.05" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-18.034" y1="0.762" x2="-18.034" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-18.034" y1="0.508" x2="-18.288" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-18.288" y1="0.508" x2="-18.288" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-18.288" y1="-0.508" x2="-18.034" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-18.034" y1="-0.508" x2="-18.034" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-18.034" y1="-0.762" x2="-17.526" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-17.526" y1="-0.762" x2="-17.526" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-17.526" y1="-0.508" x2="-17.272" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-17.272" y1="-0.508" x2="-17.272" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-17.272" y1="0.508" x2="-17.526" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-17.526" y1="0.508" x2="-17.526" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-17.526" y1="0.762" x2="-18.034" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-15.494" y1="0.762" x2="-15.494" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-15.494" y1="0.508" x2="-15.748" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-15.748" y1="0.508" x2="-15.748" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-15.748" y1="-0.508" x2="-15.494" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-15.494" y1="-0.508" x2="-15.494" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-15.494" y1="-0.762" x2="-14.986" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-14.986" y1="-0.762" x2="-14.986" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-14.986" y1="-0.508" x2="-14.732" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-14.732" y1="-0.508" x2="-14.732" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-14.732" y1="0.508" x2="-14.986" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-14.986" y1="0.508" x2="-14.986" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-14.986" y1="0.762" x2="-15.494" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-12.954" y1="0.762" x2="-12.954" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-12.954" y1="0.508" x2="-13.208" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-13.208" y1="0.508" x2="-13.208" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-13.208" y1="-0.508" x2="-12.954" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-12.954" y1="-0.508" x2="-12.954" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-12.954" y1="-0.762" x2="-12.446" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-12.446" y1="-0.762" x2="-12.446" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-12.446" y1="-0.508" x2="-12.192" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-12.192" y1="-0.508" x2="-12.192" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-12.192" y1="0.508" x2="-12.446" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-12.446" y1="0.508" x2="-12.446" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-12.446" y1="0.762" x2="-12.954" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-10.414" y1="0.762" x2="-10.414" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-10.414" y1="0.508" x2="-10.668" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-10.668" y1="0.508" x2="-10.668" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-10.668" y1="-0.508" x2="-10.414" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-10.414" y1="-0.508" x2="-10.414" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-10.414" y1="-0.762" x2="-9.906" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-9.906" y1="-0.762" x2="-9.906" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-9.906" y1="-0.508" x2="-9.652" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-9.652" y1="-0.508" x2="-9.652" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-9.652" y1="0.508" x2="-9.906" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-9.906" y1="0.508" x2="-9.906" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-9.906" y1="0.762" x2="-10.414" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-7.874" y1="0.762" x2="-7.874" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-7.874" y1="0.508" x2="-8.128" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-8.128" y1="0.508" x2="-8.128" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-8.128" y1="-0.508" x2="-7.874" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-7.874" y1="-0.508" x2="-7.874" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-7.874" y1="-0.762" x2="-7.366" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="-0.762" x2="-7.366" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="-0.508" x2="-7.112" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-7.112" y1="-0.508" x2="-7.112" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-7.112" y1="0.508" x2="-7.366" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="0.508" x2="-7.366" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="0.762" x2="-7.874" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-8.636" y1="-1.27" x2="-6.604" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-1.27" x2="-6.35" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="-1.016" x2="-6.096" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="-1.27" x2="-4.064" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.27" x2="-3.81" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.016" x2="-3.556" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.556" y1="1.27" x2="-3.81" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.016" x2="-4.064" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.27" x2="-6.096" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="1.27" x2="-6.35" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="1.016" x2="-6.604" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="1.27" x2="-8.636" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.334" y1="0.762" x2="-5.334" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-5.334" y1="0.508" x2="-5.588" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-5.588" y1="0.508" x2="-5.588" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-5.588" y1="-0.508" x2="-5.334" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-5.334" y1="-0.508" x2="-5.334" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-5.334" y1="-0.762" x2="-4.826" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="-0.762" x2="-4.826" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="-0.508" x2="-4.572" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-4.572" y1="-0.508" x2="-4.572" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-4.572" y1="0.508" x2="-4.826" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="0.508" x2="-4.826" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="0.762" x2="-5.334" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="0.762" x2="-2.794" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="0.508" x2="-3.048" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="0.508" x2="-3.048" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="-0.508" x2="-2.794" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="-0.508" x2="-2.794" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="-0.762" x2="-2.286" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="-0.762" x2="-2.286" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="-0.508" x2="-2.032" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.508" x2="-2.032" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.508" x2="-2.286" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="0.508" x2="-2.286" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="0.762" x2="-2.794" y2="0.762" width="0.1524" layer="51"/>
<wire x1="19.05" y1="1.27" x2="16.764" y2="1.27" width="0.1524" layer="21"/>
<wire x1="16.256" y1="1.27" x2="14.224" y2="1.27" width="0.1524" layer="21"/>
<wire x1="13.716" y1="1.27" x2="11.684" y2="1.27" width="0.1524" layer="21"/>
<wire x1="11.176" y1="1.27" x2="9.144" y2="1.27" width="0.1524" layer="21"/>
<wire x1="8.636" y1="1.27" x2="6.604" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.096" y1="1.27" x2="4.064" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.556" y1="1.27" x2="1.524" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.016" y1="1.27" x2="-1.016" y2="1.27" width="0.1524" layer="21"/>
<wire x1="19.05" y1="-1.27" x2="19.05" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.016" y1="-1.27" x2="1.016" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-1.27" x2="3.556" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.27" x2="6.096" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.604" y1="-1.27" x2="8.636" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="9.144" y1="-1.27" x2="11.176" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="11.684" y1="-1.27" x2="13.716" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="14.224" y1="-1.27" x2="16.256" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="16.764" y1="-1.27" x2="19.05" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.016" x2="-1.524" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.016" y1="1.27" x2="-1.27" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="-3.556" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.016" x2="-1.016" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-1.27" x2="-1.27" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.556" y1="-1.27" x2="-1.524" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="0.508" x2="-0.508" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="0.508" x2="-0.508" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="-0.508" x2="-0.254" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="-0.508" x2="-0.254" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="-0.762" x2="0.254" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="0.254" y1="-0.762" x2="0.254" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="0.254" y1="-0.508" x2="0.508" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-0.508" x2="0.508" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0.508" x2="0.254" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.254" y1="0.508" x2="0.254" y2="0.762" width="0.1524" layer="51"/>
<wire x1="0.254" y1="0.762" x2="-0.254" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.524" y1="1.27" x2="1.27" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.016" x2="1.016" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.016" x2="1.524" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.016" y1="-1.27" x2="1.27" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.016" x2="4.064" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.556" y1="-1.27" x2="3.81" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.27" x2="3.81" y2="1.016" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.016" x2="3.556" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.604" y1="1.27" x2="6.35" y2="1.016" width="0.1524" layer="21"/>
<wire x1="6.35" y1="1.016" x2="6.096" y2="1.27" width="0.1524" layer="21"/>
<wire x1="9.144" y1="1.27" x2="8.89" y2="1.016" width="0.1524" layer="21"/>
<wire x1="8.89" y1="1.016" x2="8.636" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.35" y1="-1.016" x2="6.604" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-1.27" x2="6.35" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="8.89" y1="-1.016" x2="9.144" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-1.27" x2="8.89" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="0.762" x2="2.286" y2="0.508" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0.508" x2="2.032" y2="0.508" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.508" x2="2.032" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.508" x2="2.286" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.286" y1="-0.508" x2="2.286" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="2.286" y1="-0.762" x2="2.794" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="2.794" y1="-0.762" x2="2.794" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.794" y1="-0.508" x2="3.048" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="3.048" y1="-0.508" x2="3.048" y2="0.508" width="0.1524" layer="51"/>
<wire x1="3.048" y1="0.508" x2="2.794" y2="0.508" width="0.1524" layer="51"/>
<wire x1="2.794" y1="0.508" x2="2.794" y2="0.762" width="0.1524" layer="51"/>
<wire x1="2.794" y1="0.762" x2="2.286" y2="0.762" width="0.1524" layer="51"/>
<wire x1="4.826" y1="0.762" x2="4.826" y2="0.508" width="0.1524" layer="51"/>
<wire x1="4.826" y1="0.508" x2="4.572" y2="0.508" width="0.1524" layer="51"/>
<wire x1="4.572" y1="0.508" x2="4.572" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="4.572" y1="-0.508" x2="4.826" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="4.826" y1="-0.508" x2="4.826" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="4.826" y1="-0.762" x2="5.334" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="5.334" y1="-0.762" x2="5.334" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="5.334" y1="-0.508" x2="5.588" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="5.588" y1="-0.508" x2="5.588" y2="0.508" width="0.1524" layer="51"/>
<wire x1="5.588" y1="0.508" x2="5.334" y2="0.508" width="0.1524" layer="51"/>
<wire x1="5.334" y1="0.508" x2="5.334" y2="0.762" width="0.1524" layer="51"/>
<wire x1="5.334" y1="0.762" x2="4.826" y2="0.762" width="0.1524" layer="51"/>
<wire x1="7.366" y1="0.762" x2="7.366" y2="0.508" width="0.1524" layer="51"/>
<wire x1="7.366" y1="0.508" x2="7.112" y2="0.508" width="0.1524" layer="51"/>
<wire x1="7.112" y1="0.508" x2="7.112" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="7.112" y1="-0.508" x2="7.366" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="7.366" y1="-0.508" x2="7.366" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="7.366" y1="-0.762" x2="7.874" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="7.874" y1="-0.762" x2="7.874" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="7.874" y1="-0.508" x2="8.128" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="8.128" y1="-0.508" x2="8.128" y2="0.508" width="0.1524" layer="51"/>
<wire x1="8.128" y1="0.508" x2="7.874" y2="0.508" width="0.1524" layer="51"/>
<wire x1="7.874" y1="0.508" x2="7.874" y2="0.762" width="0.1524" layer="51"/>
<wire x1="7.874" y1="0.762" x2="7.366" y2="0.762" width="0.1524" layer="51"/>
<wire x1="9.906" y1="0.762" x2="9.906" y2="0.508" width="0.1524" layer="51"/>
<wire x1="9.906" y1="0.508" x2="9.652" y2="0.508" width="0.1524" layer="51"/>
<wire x1="9.652" y1="0.508" x2="9.652" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="9.652" y1="-0.508" x2="9.906" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="9.906" y1="-0.508" x2="9.906" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="9.906" y1="-0.762" x2="10.414" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="10.414" y1="-0.762" x2="10.414" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="10.414" y1="-0.508" x2="10.668" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="10.668" y1="-0.508" x2="10.668" y2="0.508" width="0.1524" layer="51"/>
<wire x1="10.668" y1="0.508" x2="10.414" y2="0.508" width="0.1524" layer="51"/>
<wire x1="10.414" y1="0.508" x2="10.414" y2="0.762" width="0.1524" layer="51"/>
<wire x1="10.414" y1="0.762" x2="9.906" y2="0.762" width="0.1524" layer="51"/>
<wire x1="11.684" y1="1.27" x2="11.43" y2="1.016" width="0.1524" layer="21"/>
<wire x1="11.43" y1="1.016" x2="11.176" y2="1.27" width="0.1524" layer="21"/>
<wire x1="11.43" y1="-1.016" x2="11.684" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="11.176" y1="-1.27" x2="11.43" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="12.446" y1="0.762" x2="12.446" y2="0.508" width="0.1524" layer="51"/>
<wire x1="12.446" y1="0.508" x2="12.192" y2="0.508" width="0.1524" layer="51"/>
<wire x1="12.192" y1="0.508" x2="12.192" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="12.192" y1="-0.508" x2="12.446" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="12.446" y1="-0.508" x2="12.446" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="12.446" y1="-0.762" x2="12.954" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="12.954" y1="-0.762" x2="12.954" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="12.954" y1="-0.508" x2="13.208" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="13.208" y1="-0.508" x2="13.208" y2="0.508" width="0.1524" layer="51"/>
<wire x1="13.208" y1="0.508" x2="12.954" y2="0.508" width="0.1524" layer="51"/>
<wire x1="12.954" y1="0.508" x2="12.954" y2="0.762" width="0.1524" layer="51"/>
<wire x1="12.954" y1="0.762" x2="12.446" y2="0.762" width="0.1524" layer="51"/>
<wire x1="14.224" y1="1.27" x2="13.97" y2="1.016" width="0.1524" layer="21"/>
<wire x1="16.764" y1="1.27" x2="16.51" y2="1.016" width="0.1524" layer="21"/>
<wire x1="13.97" y1="1.016" x2="13.716" y2="1.27" width="0.1524" layer="21"/>
<wire x1="16.51" y1="1.016" x2="16.256" y2="1.27" width="0.1524" layer="21"/>
<wire x1="13.97" y1="-1.016" x2="14.224" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="16.51" y1="-1.016" x2="16.764" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="13.716" y1="-1.27" x2="13.97" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="16.256" y1="-1.27" x2="16.51" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="14.986" y1="0.762" x2="14.986" y2="0.508" width="0.1524" layer="51"/>
<wire x1="14.986" y1="0.508" x2="14.732" y2="0.508" width="0.1524" layer="51"/>
<wire x1="14.732" y1="0.508" x2="14.732" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="14.732" y1="-0.508" x2="14.986" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="14.986" y1="-0.508" x2="14.986" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="14.986" y1="-0.762" x2="15.494" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="15.494" y1="-0.762" x2="15.494" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="15.494" y1="-0.508" x2="15.748" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="15.748" y1="-0.508" x2="15.748" y2="0.508" width="0.1524" layer="51"/>
<wire x1="15.748" y1="0.508" x2="15.494" y2="0.508" width="0.1524" layer="51"/>
<wire x1="15.494" y1="0.508" x2="15.494" y2="0.762" width="0.1524" layer="51"/>
<wire x1="15.494" y1="0.762" x2="14.986" y2="0.762" width="0.1524" layer="51"/>
<wire x1="17.526" y1="0.762" x2="17.526" y2="0.508" width="0.1524" layer="51"/>
<wire x1="17.526" y1="0.508" x2="17.272" y2="0.508" width="0.1524" layer="51"/>
<wire x1="17.272" y1="0.508" x2="17.272" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="17.272" y1="-0.508" x2="17.526" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="17.526" y1="-0.508" x2="17.526" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="17.526" y1="-0.762" x2="18.034" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="18.034" y1="-0.762" x2="18.034" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="18.034" y1="-0.508" x2="18.288" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="18.288" y1="-0.508" x2="18.288" y2="0.508" width="0.1524" layer="51"/>
<wire x1="18.288" y1="0.508" x2="18.034" y2="0.508" width="0.1524" layer="51"/>
<wire x1="18.034" y1="0.508" x2="18.034" y2="0.762" width="0.1524" layer="51"/>
<wire x1="18.034" y1="0.762" x2="17.526" y2="0.762" width="0.1524" layer="51"/>
<pad name="1" x="-17.78" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-15.24" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-12.7" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="-10.16" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="-7.62" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="-5.08" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="-2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="0" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="5.08" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="7.62" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="10.16" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="12.7" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="15.24" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="15" x="17.78" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-15.24" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-18.288" y="1.524" size="1.27" layer="21" ratio="10">1</text>
<text x="-19.05" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="16.637" y="1.524" size="1.27" layer="21" ratio="10">15</text>
<rectangle x1="-17.907" y1="0.254" x2="-17.653" y2="0.762" layer="51"/>
<rectangle x1="-17.907" y1="-0.762" x2="-17.653" y2="-0.254" layer="51"/>
<rectangle x1="-15.367" y1="0.254" x2="-15.113" y2="0.762" layer="51"/>
<rectangle x1="-15.367" y1="-0.762" x2="-15.113" y2="-0.254" layer="51"/>
<rectangle x1="-12.827" y1="0.254" x2="-12.573" y2="0.762" layer="51"/>
<rectangle x1="-12.827" y1="-0.762" x2="-12.573" y2="-0.254" layer="51"/>
<rectangle x1="-10.287" y1="0.254" x2="-10.033" y2="0.762" layer="51"/>
<rectangle x1="-10.287" y1="-0.762" x2="-10.033" y2="-0.254" layer="51"/>
<rectangle x1="-7.747" y1="0.254" x2="-7.493" y2="0.762" layer="51"/>
<rectangle x1="-7.747" y1="-0.762" x2="-7.493" y2="-0.254" layer="51"/>
<rectangle x1="-5.207" y1="0.254" x2="-4.953" y2="0.762" layer="51"/>
<rectangle x1="-5.207" y1="-0.762" x2="-4.953" y2="-0.254" layer="51"/>
<rectangle x1="-2.667" y1="0.254" x2="-2.413" y2="0.762" layer="51"/>
<rectangle x1="-2.667" y1="-0.762" x2="-2.413" y2="-0.254" layer="51"/>
<rectangle x1="-0.127" y1="0.254" x2="0.127" y2="0.762" layer="51"/>
<rectangle x1="-0.127" y1="-0.762" x2="0.127" y2="-0.254" layer="51"/>
<rectangle x1="2.413" y1="0.254" x2="2.667" y2="0.762" layer="51"/>
<rectangle x1="2.413" y1="-0.762" x2="2.667" y2="-0.254" layer="51"/>
<rectangle x1="4.953" y1="0.254" x2="5.207" y2="0.762" layer="51"/>
<rectangle x1="4.953" y1="-0.762" x2="5.207" y2="-0.254" layer="51"/>
<rectangle x1="7.493" y1="0.254" x2="7.747" y2="0.762" layer="51"/>
<rectangle x1="7.493" y1="-0.762" x2="7.747" y2="-0.254" layer="51"/>
<rectangle x1="10.033" y1="0.254" x2="10.287" y2="0.762" layer="51"/>
<rectangle x1="10.033" y1="-0.762" x2="10.287" y2="-0.254" layer="51"/>
<rectangle x1="12.573" y1="0.254" x2="12.827" y2="0.762" layer="51"/>
<rectangle x1="12.573" y1="-0.762" x2="12.827" y2="-0.254" layer="51"/>
<rectangle x1="15.113" y1="0.254" x2="15.367" y2="0.762" layer="51"/>
<rectangle x1="15.113" y1="-0.762" x2="15.367" y2="-0.254" layer="51"/>
<rectangle x1="17.653" y1="0.254" x2="17.907" y2="0.762" layer="51"/>
<rectangle x1="17.653" y1="-0.762" x2="17.907" y2="-0.254" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="FE15" urn="urn:adsk.eagle:package:8206/1" type="box" library_version="2">
<description>FEMALE HEADER</description>
<packageinstances>
<packageinstance name="FE15"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="FE15-1" urn="urn:adsk.eagle:symbol:8164/1" library_version="2">
<wire x1="3.81" y1="-20.32" x2="-1.27" y2="-20.32" width="0.4064" layer="94"/>
<wire x1="1.905" y1="-12.065" x2="1.905" y2="-13.335" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-14.605" x2="1.905" y2="-15.875" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-17.145" x2="1.905" y2="-18.415" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-6.985" x2="1.905" y2="-8.255" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-9.525" x2="1.905" y2="-10.795" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="0.635" x2="1.905" y2="-0.635" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-1.905" x2="1.905" y2="-3.175" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-4.445" x2="1.905" y2="-5.715" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="5.715" x2="1.905" y2="4.445" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="3.175" x2="1.905" y2="1.905" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="13.335" x2="1.905" y2="12.065" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="10.795" x2="1.905" y2="9.525" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="8.255" x2="1.905" y2="6.985" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="18.415" x2="1.905" y2="17.145" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="15.875" x2="1.905" y2="14.605" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="-1.27" y1="20.32" x2="-1.27" y2="-20.32" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-20.32" x2="3.81" y2="20.32" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="20.32" x2="3.81" y2="20.32" width="0.4064" layer="94"/>
<text x="-1.27" y="-22.86" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="21.082" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-17.78" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="-15.24" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="-12.7" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="7.62" y="-10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="-7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="6" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="7" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="8" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="9" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="10" x="7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="11" x="7.62" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="12" x="7.62" y="10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="13" x="7.62" y="12.7" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="14" x="7.62" y="15.24" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="15" x="7.62" y="17.78" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="FE15-1" urn="urn:adsk.eagle:component:8252/2" prefix="SV" uservalue="yes" library_version="2">
<description>&lt;b&gt;FEMALE HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="FE15-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="FE15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="11" pad="11"/>
<connect gate="G$1" pin="12" pad="12"/>
<connect gate="G$1" pin="13" pad="13"/>
<connect gate="G$1" pin="14" pad="14"/>
<connect gate="G$1" pin="15" pad="15"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8206/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="2" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="switch" urn="urn:adsk.eagle:library:380">
<description>&lt;b&gt;Switches&lt;/b&gt;&lt;p&gt;
Marquardt, Siemens, C&amp;K, ITT, and others&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="TL3XPO" urn="urn:adsk.eagle:footprint:27631/1" library_version="3">
<description>&lt;b&gt;TINY SWITCH&lt;/b&gt;&lt;p&gt;
Source: http://www2.produktinfo.conrad.com/datenblaetter/700000-724999/705152-da-01-de-Subminiaturschalter_TL_36YO.pdf</description>
<wire x1="-5.08" y1="2.54" x2="5.08" y2="2.54" width="0.2032" layer="21"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-2.54" width="0.2032" layer="21"/>
<wire x1="5.08" y1="-2.54" x2="-5.08" y2="-2.54" width="0.2032" layer="21"/>
<wire x1="-5.08" y1="-2.54" x2="-5.08" y2="2.54" width="0.2032" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="0" y2="1.27" width="0.2032" layer="51"/>
<wire x1="-3.175" y1="-1.27" x2="0" y2="-1.27" width="0.2032" layer="51"/>
<wire x1="0" y1="1.27" x2="0" y2="-1.27" width="0.2032" layer="51" curve="-180"/>
<wire x1="-1.375" y1="1.275" x2="-1.35" y2="-1.3" width="0.2032" layer="51" curve="-273.242292"/>
<circle x="-3.175" y="0" radius="1.27" width="0.2032" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.1" diameter="1.4224" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.1" diameter="1.4224" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.1" diameter="1.4224" shape="long" rot="R90"/>
<text x="-5.08" y="3.175" size="1.778" layer="25">&gt;NAME</text>
<text x="-5.08" y="-5.08" size="1.778" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="TL3XPO" urn="urn:adsk.eagle:package:27745/1" type="box" library_version="3">
<description>TINY SWITCH
Source: http://www2.produktinfo.conrad.com/datenblaetter/700000-724999/705152-da-01-de-Subminiaturschalter_TL_36YO.pdf</description>
<packageinstances>
<packageinstance name="TL3XPO"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="U" urn="urn:adsk.eagle:symbol:27630/1" library_version="3">
<wire x1="0" y1="-3.175" x2="0" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="-1.905" x2="-1.905" y2="3.175" width="0.254" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.254" layer="94"/>
<wire x1="2.54" y1="2.54" x2="2.54" y2="3.175" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-2.54" y2="3.175" width="0.254" layer="94"/>
<text x="5.08" y="-2.54" size="1.778" layer="95" rot="R90">&gt;NAME</text>
<text x="7.62" y="-2.54" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="P" x="0" y="-5.08" visible="pad" length="short" direction="pas" rot="R90"/>
<pin name="S" x="2.54" y="5.08" visible="pad" length="short" direction="pas" rot="R270"/>
<pin name="O" x="-2.54" y="5.08" visible="pad" length="short" direction="pas" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="TL36PO" urn="urn:adsk.eagle:component:27817/2" prefix="S" library_version="3">
<description>&lt;b&gt;TINY SWITCH&lt;/b&gt; ON - ON&lt;p&gt;
Source: http://www2.produktinfo.conrad.com/datenblaetter/700000-724999/705152-da-01-de-Subminiaturschalter_TL_36YO.pdf</description>
<gates>
<gate name="BEF1" symbol="U" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TL3XPO">
<connects>
<connect gate="BEF1" pin="O" pad="1"/>
<connect gate="BEF1" pin="P" pad="2"/>
<connect gate="BEF1" pin="S" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:27745/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
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
<part name="LIDAR" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-?-07" device="27-2071" package3d_urn="urn:adsk.eagle:package:8078655/1"/>
<part name="MOTOR" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2081" device="" package3d_urn="urn:adsk.eagle:package:8078639/1"/>
<part name="CAM" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2041" device="" package3d_urn="urn:adsk.eagle:package:8078635/1"/>
<part name="CN4" library="con-lsta" library_urn="urn:adsk.eagle:library:161" deviceset="FE15-1" device="" package3d_urn="urn:adsk.eagle:package:8206/1"/>
<part name="CN3" library="con-lsta" library_urn="urn:adsk.eagle:library:161" deviceset="FE15-1" device="" package3d_urn="urn:adsk.eagle:package:8206/1"/>
<part name="S1" library="switch" library_urn="urn:adsk.eagle:library:380" deviceset="TL36PO" device="" package3d_urn="urn:adsk.eagle:package:27745/1"/>
<part name="COL" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2051" device="" package3d_urn="urn:adsk.eagle:package:8078636/1"/>
<part name="12V" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2021" device="" package3d_urn="urn:adsk.eagle:package:8078633/1"/>
<part name="5V" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2021" device="" package3d_urn="urn:adsk.eagle:package:8078633/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="LIDAR" gate="-1" x="5.08" y="83.82" smashed="yes" rot="MR0">
<attribute name="NAME" x="2.54" y="83.058" size="1.524" layer="95" rot="MR0"/>
<attribute name="VALUE" x="5.842" y="85.217" size="1.778" layer="96" rot="MR0"/>
</instance>
<instance part="LIDAR" gate="-2" x="5.08" y="78.74" smashed="yes" rot="MR0">
<attribute name="NAME" x="2.54" y="77.978" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="LIDAR" gate="-3" x="5.08" y="73.66" smashed="yes" rot="MR0">
<attribute name="NAME" x="2.54" y="72.898" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="LIDAR" gate="-4" x="5.08" y="68.58" smashed="yes" rot="MR0">
<attribute name="NAME" x="2.54" y="67.818" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="LIDAR" gate="-5" x="5.08" y="63.5" smashed="yes" rot="MR0">
<attribute name="NAME" x="2.54" y="62.738" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="LIDAR" gate="-6" x="5.08" y="58.42" smashed="yes" rot="MR0">
<attribute name="NAME" x="2.54" y="57.658" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="LIDAR" gate="-7" x="5.08" y="53.34" smashed="yes" rot="MR0">
<attribute name="NAME" x="2.54" y="52.578" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-1" x="10.16" y="114.3" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="113.538" size="1.524" layer="95" rot="MR0"/>
<attribute name="VALUE" x="10.922" y="115.697" size="1.778" layer="96" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-2" x="10.16" y="111.76" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="110.998" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-3" x="10.16" y="109.22" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="108.458" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-4" x="10.16" y="106.68" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="105.918" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-5" x="10.16" y="104.14" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="103.378" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-6" x="10.16" y="101.6" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="100.838" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-7" x="10.16" y="99.06" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="98.298" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="MOTOR" gate="-8" x="10.16" y="96.52" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="95.758" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="CAM" gate="-1" x="10.16" y="20.32" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="19.558" size="1.524" layer="95" rot="MR0"/>
<attribute name="VALUE" x="10.922" y="21.717" size="1.778" layer="96" rot="MR0"/>
</instance>
<instance part="CAM" gate="-2" x="10.16" y="17.78" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="17.018" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="CAM" gate="-3" x="10.16" y="15.24" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="14.478" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="CAM" gate="-4" x="10.16" y="12.7" smashed="yes" rot="MR0">
<attribute name="NAME" x="7.62" y="11.938" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="CN4" gate="G$1" x="68.58" y="71.12" smashed="yes">
<attribute name="VALUE" x="67.31" y="48.26" size="1.778" layer="96"/>
<attribute name="NAME" x="67.31" y="92.202" size="1.778" layer="95"/>
</instance>
<instance part="CN3" gate="G$1" x="50.8" y="71.12" smashed="yes" rot="MR0">
<attribute name="VALUE" x="52.07" y="48.26" size="1.778" layer="96" rot="MR0"/>
<attribute name="NAME" x="52.07" y="92.202" size="1.778" layer="95" rot="MR0"/>
</instance>
<instance part="S1" gate="BEF1" x="15.24" y="127" smashed="yes" rot="R90">
<attribute name="NAME" x="17.78" y="132.08" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="17.78" y="134.62" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="COL" gate="-1" x="0" y="43.18" smashed="yes" rot="MR0">
<attribute name="NAME" x="-2.54" y="42.418" size="1.524" layer="95" rot="MR0"/>
<attribute name="VALUE" x="0.762" y="44.577" size="1.778" layer="96" rot="MR0"/>
</instance>
<instance part="COL" gate="-2" x="0" y="40.64" smashed="yes" rot="MR0">
<attribute name="NAME" x="-2.54" y="39.878" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="COL" gate="-3" x="0" y="38.1" smashed="yes" rot="MR0">
<attribute name="NAME" x="-2.54" y="37.338" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="COL" gate="-4" x="0" y="35.56" smashed="yes" rot="MR0">
<attribute name="NAME" x="-2.54" y="34.798" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="COL" gate="-5" x="0" y="33.02" smashed="yes" rot="MR0">
<attribute name="NAME" x="-2.54" y="32.258" size="1.524" layer="95" rot="MR0"/>
</instance>
<instance part="12V" gate="-1" x="63.5" y="114.3" smashed="yes">
<attribute name="NAME" x="66.04" y="113.538" size="1.524" layer="95"/>
<attribute name="VALUE" x="62.738" y="115.697" size="1.778" layer="96"/>
</instance>
<instance part="12V" gate="-2" x="63.5" y="111.76" smashed="yes">
<attribute name="NAME" x="66.04" y="110.998" size="1.524" layer="95"/>
</instance>
<instance part="5V" gate="-1" x="63.5" y="106.68" smashed="yes">
<attribute name="NAME" x="66.04" y="105.918" size="1.524" layer="95"/>
<attribute name="VALUE" x="62.738" y="108.077" size="1.778" layer="96"/>
</instance>
<instance part="5V" gate="-2" x="63.5" y="104.14" smashed="yes">
<attribute name="NAME" x="66.04" y="103.378" size="1.524" layer="95"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="14"/>
<wire x1="76.2" y1="86.36" x2="91.44" y2="86.36" width="0.1524" layer="91"/>
<wire x1="91.44" y1="86.36" x2="91.44" y2="99.06" width="0.1524" layer="91"/>
<wire x1="91.44" y1="99.06" x2="40.64" y2="99.06" width="0.1524" layer="91"/>
<wire x1="40.64" y1="99.06" x2="40.64" y2="109.22" width="0.1524" layer="91"/>
<wire x1="91.44" y1="86.36" x2="91.44" y2="5.08" width="0.1524" layer="91"/>
<wire x1="91.44" y1="5.08" x2="15.24" y2="5.08" width="0.1524" layer="91"/>
<junction x="91.44" y="86.36"/>
<wire x1="15.24" y1="5.08" x2="-10.16" y2="5.08" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="5.08" x2="-10.16" y2="30.48" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="30.48" x2="-10.16" y2="50.8" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="50.8" x2="-10.16" y2="88.9" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="88.9" x2="20.32" y2="88.9" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-8" pin="S"/>
<wire x1="20.32" y1="88.9" x2="20.32" y2="96.52" width="0.1524" layer="91"/>
<wire x1="20.32" y1="96.52" x2="12.7" y2="96.52" width="0.1524" layer="91"/>
<pinref part="LIDAR" gate="-5" pin="S"/>
<wire x1="7.62" y1="63.5" x2="12.7" y2="63.5" width="0.1524" layer="91"/>
<wire x1="12.7" y1="63.5" x2="12.7" y2="50.8" width="0.1524" layer="91"/>
<wire x1="12.7" y1="50.8" x2="-10.16" y2="50.8" width="0.1524" layer="91"/>
<junction x="-10.16" y="50.8"/>
<pinref part="LIDAR" gate="-4" pin="S"/>
<wire x1="7.62" y1="68.58" x2="12.7" y2="68.58" width="0.1524" layer="91"/>
<wire x1="12.7" y1="68.58" x2="12.7" y2="63.5" width="0.1524" layer="91"/>
<junction x="12.7" y="63.5"/>
<wire x1="17.78" y1="33.02" x2="17.78" y2="30.48" width="0.1524" layer="91"/>
<wire x1="17.78" y1="30.48" x2="-10.16" y2="30.48" width="0.1524" layer="91"/>
<junction x="-10.16" y="30.48"/>
<pinref part="CAM" gate="-4" pin="S"/>
<wire x1="12.7" y1="12.7" x2="15.24" y2="12.7" width="0.1524" layer="91"/>
<wire x1="15.24" y1="12.7" x2="15.24" y2="5.08" width="0.1524" layer="91"/>
<junction x="15.24" y="5.08"/>
<wire x1="17.78" y1="33.02" x2="5.08" y2="33.02" width="0.1524" layer="91"/>
<wire x1="5.08" y1="33.02" x2="5.08" y2="35.56" width="0.1524" layer="91"/>
<pinref part="COL" gate="-4" pin="S"/>
<wire x1="5.08" y1="35.56" x2="2.54" y2="35.56" width="0.1524" layer="91"/>
<pinref part="COL" gate="-5" pin="S"/>
<wire x1="2.54" y1="33.02" x2="5.08" y2="33.02" width="0.1524" layer="91"/>
<junction x="5.08" y="33.02"/>
<wire x1="40.64" y1="109.22" x2="48.26" y2="109.22" width="0.1524" layer="91"/>
<pinref part="12V" gate="-2" pin="S"/>
<wire x1="48.26" y1="109.22" x2="60.96" y2="109.22" width="0.1524" layer="91"/>
<wire x1="60.96" y1="109.22" x2="60.96" y2="111.76" width="0.1524" layer="91"/>
<pinref part="5V" gate="-2" pin="S"/>
<wire x1="48.26" y1="109.22" x2="48.26" y2="104.14" width="0.1524" layer="91"/>
<wire x1="48.26" y1="104.14" x2="60.96" y2="104.14" width="0.1524" layer="91"/>
<junction x="48.26" y="109.22"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="12"/>
<wire x1="76.2" y1="81.28" x2="88.9" y2="81.28" width="0.1524" layer="91"/>
<wire x1="88.9" y1="81.28" x2="88.9" y2="96.52" width="0.1524" layer="91"/>
<wire x1="88.9" y1="96.52" x2="38.1" y2="96.52" width="0.1524" layer="91"/>
<wire x1="38.1" y1="96.52" x2="38.1" y2="111.76" width="0.1524" layer="91"/>
<wire x1="88.9" y1="81.28" x2="88.9" y2="7.62" width="0.1524" layer="91"/>
<junction x="88.9" y="81.28"/>
<wire x1="88.9" y1="7.62" x2="17.78" y2="7.62" width="0.1524" layer="91"/>
<wire x1="17.78" y1="7.62" x2="-7.62" y2="7.62" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="7.62" x2="-7.62" y2="27.94" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="27.94" x2="-7.62" y2="48.26" width="0.1524" layer="91"/>
<wire x1="7.62" y1="35.56" x2="20.32" y2="35.56" width="0.1524" layer="91"/>
<wire x1="20.32" y1="35.56" x2="20.32" y2="27.94" width="0.1524" layer="91"/>
<wire x1="20.32" y1="27.94" x2="-7.62" y2="27.94" width="0.1524" layer="91"/>
<junction x="-7.62" y="27.94"/>
<pinref part="CAM" gate="-3" pin="S"/>
<wire x1="12.7" y1="15.24" x2="17.78" y2="15.24" width="0.1524" layer="91"/>
<wire x1="17.78" y1="15.24" x2="17.78" y2="7.62" width="0.1524" layer="91"/>
<junction x="17.78" y="7.62"/>
<wire x1="-7.62" y1="48.26" x2="10.16" y2="48.26" width="0.1524" layer="91"/>
<pinref part="LIDAR" gate="-7" pin="S"/>
<pinref part="LIDAR" gate="-6" pin="S"/>
<wire x1="7.62" y1="58.42" x2="10.16" y2="58.42" width="0.1524" layer="91"/>
<wire x1="10.16" y1="58.42" x2="10.16" y2="53.34" width="0.1524" layer="91"/>
<wire x1="10.16" y1="53.34" x2="7.62" y2="53.34" width="0.1524" layer="91"/>
<wire x1="10.16" y1="48.26" x2="10.16" y2="53.34" width="0.1524" layer="91"/>
<junction x="10.16" y="53.34"/>
<wire x1="7.62" y1="35.56" x2="7.62" y2="38.1" width="0.1524" layer="91"/>
<pinref part="COL" gate="-3" pin="S"/>
<wire x1="7.62" y1="38.1" x2="2.54" y2="38.1" width="0.1524" layer="91"/>
<wire x1="38.1" y1="111.76" x2="50.8" y2="111.76" width="0.1524" layer="91"/>
<wire x1="50.8" y1="111.76" x2="50.8" y2="106.68" width="0.1524" layer="91"/>
<pinref part="5V" gate="-1" pin="S"/>
<wire x1="50.8" y1="106.68" x2="60.96" y2="106.68" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<wire x1="-7.62" y1="129.54" x2="-7.62" y2="91.44" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="91.44" x2="15.24" y2="91.44" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-7" pin="S"/>
<wire x1="15.24" y1="91.44" x2="15.24" y2="99.06" width="0.1524" layer="91"/>
<wire x1="15.24" y1="99.06" x2="12.7" y2="99.06" width="0.1524" layer="91"/>
<pinref part="S1" gate="BEF1" pin="S"/>
<wire x1="-7.62" y1="129.54" x2="10.16" y2="129.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<wire x1="40.64" y1="114.3" x2="40.64" y2="127" width="0.1524" layer="91"/>
<pinref part="S1" gate="BEF1" pin="P"/>
<wire x1="20.32" y1="127" x2="40.64" y2="127" width="0.1524" layer="91"/>
<pinref part="12V" gate="-1" pin="S"/>
<wire x1="40.64" y1="114.3" x2="60.96" y2="114.3" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<wire x1="35.56" y1="114.3" x2="35.56" y2="78.74" width="0.1524" layer="91"/>
<pinref part="CN3" gate="G$1" pin="11"/>
<wire x1="35.56" y1="78.74" x2="43.18" y2="78.74" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-1" pin="S"/>
<wire x1="35.56" y1="114.3" x2="12.7" y2="114.3" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="CN3" gate="G$1" pin="8"/>
<wire x1="43.18" y1="71.12" x2="27.94" y2="71.12" width="0.1524" layer="91"/>
<wire x1="27.94" y1="71.12" x2="27.94" y2="106.68" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-4" pin="S"/>
<wire x1="27.94" y1="106.68" x2="12.7" y2="106.68" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="CN3" gate="G$1" pin="7"/>
<wire x1="43.18" y1="68.58" x2="25.4" y2="68.58" width="0.1524" layer="91"/>
<wire x1="25.4" y1="68.58" x2="25.4" y2="104.14" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-5" pin="S"/>
<wire x1="25.4" y1="104.14" x2="12.7" y2="104.14" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="CN3" gate="G$1" pin="6"/>
<wire x1="43.18" y1="66.04" x2="22.86" y2="66.04" width="0.1524" layer="91"/>
<wire x1="22.86" y1="66.04" x2="22.86" y2="101.6" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-6" pin="S"/>
<wire x1="22.86" y1="101.6" x2="12.7" y2="101.6" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="11"/>
<wire x1="76.2" y1="78.74" x2="86.36" y2="78.74" width="0.1524" layer="91"/>
<wire x1="86.36" y1="78.74" x2="86.36" y2="48.26" width="0.1524" layer="91"/>
<wire x1="86.36" y1="48.26" x2="20.32" y2="48.26" width="0.1524" layer="91"/>
<wire x1="20.32" y1="48.26" x2="20.32" y2="83.82" width="0.1524" layer="91"/>
<pinref part="LIDAR" gate="-1" pin="S"/>
<wire x1="20.32" y1="83.82" x2="7.62" y2="83.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="6"/>
<wire x1="76.2" y1="66.04" x2="83.82" y2="66.04" width="0.1524" layer="91"/>
<wire x1="83.82" y1="66.04" x2="83.82" y2="45.72" width="0.1524" layer="91"/>
<wire x1="83.82" y1="45.72" x2="17.78" y2="45.72" width="0.1524" layer="91"/>
<pinref part="LIDAR" gate="-2" pin="S"/>
<wire x1="17.78" y1="45.72" x2="17.78" y2="78.74" width="0.1524" layer="91"/>
<wire x1="17.78" y1="78.74" x2="7.62" y2="78.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="5"/>
<wire x1="76.2" y1="63.5" x2="81.28" y2="63.5" width="0.1524" layer="91"/>
<wire x1="81.28" y1="63.5" x2="81.28" y2="43.18" width="0.1524" layer="91"/>
<wire x1="81.28" y1="43.18" x2="15.24" y2="43.18" width="0.1524" layer="91"/>
<wire x1="15.24" y1="43.18" x2="15.24" y2="73.66" width="0.1524" layer="91"/>
<pinref part="LIDAR" gate="-3" pin="S"/>
<wire x1="15.24" y1="73.66" x2="7.62" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<wire x1="12.7" y1="40.64" x2="78.74" y2="40.64" width="0.1524" layer="91"/>
<pinref part="CN4" gate="G$1" pin="10"/>
<wire x1="78.74" y1="40.64" x2="78.74" y2="76.2" width="0.1524" layer="91"/>
<wire x1="78.74" y1="76.2" x2="76.2" y2="76.2" width="0.1524" layer="91"/>
<pinref part="CAM" gate="-1" pin="S"/>
<wire x1="12.7" y1="20.32" x2="78.74" y2="20.32" width="0.1524" layer="91"/>
<wire x1="78.74" y1="20.32" x2="78.74" y2="40.64" width="0.1524" layer="91"/>
<junction x="78.74" y="40.64"/>
<pinref part="COL" gate="-1" pin="S"/>
<wire x1="12.7" y1="40.64" x2="12.7" y2="43.18" width="0.1524" layer="91"/>
<wire x1="12.7" y1="43.18" x2="2.54" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<wire x1="10.16" y1="38.1" x2="40.64" y2="38.1" width="0.1524" layer="91"/>
<pinref part="CN3" gate="G$1" pin="1"/>
<wire x1="40.64" y1="38.1" x2="40.64" y2="53.34" width="0.1524" layer="91"/>
<wire x1="40.64" y1="53.34" x2="43.18" y2="53.34" width="0.1524" layer="91"/>
<pinref part="CAM" gate="-2" pin="S"/>
<wire x1="12.7" y1="17.78" x2="40.64" y2="17.78" width="0.1524" layer="91"/>
<wire x1="40.64" y1="17.78" x2="40.64" y2="38.1" width="0.1524" layer="91"/>
<junction x="40.64" y="38.1"/>
<wire x1="10.16" y1="38.1" x2="10.16" y2="40.64" width="0.1524" layer="91"/>
<pinref part="COL" gate="-2" pin="S"/>
<wire x1="10.16" y1="40.64" x2="2.54" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="CN3" gate="G$1" pin="10"/>
<wire x1="43.18" y1="76.2" x2="33.02" y2="76.2" width="0.1524" layer="91"/>
<wire x1="33.02" y1="76.2" x2="33.02" y2="111.76" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-2" pin="S"/>
<wire x1="33.02" y1="111.76" x2="12.7" y2="111.76" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="CN3" gate="G$1" pin="9"/>
<wire x1="43.18" y1="73.66" x2="30.48" y2="73.66" width="0.1524" layer="91"/>
<wire x1="30.48" y1="73.66" x2="30.48" y2="109.22" width="0.1524" layer="91"/>
<pinref part="MOTOR" gate="-3" pin="S"/>
<wire x1="30.48" y1="109.22" x2="12.7" y2="109.22" width="0.1524" layer="91"/>
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
