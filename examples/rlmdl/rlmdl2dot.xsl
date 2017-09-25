<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:output method="text" omit-xml-declaration="yes"/>
	<xsl:strip-space elements="*"/>
	<xsl:template match="/rl/mdl/model|/rlmdl/model">
		<xsl:text>digraph G {</xsl:text>
		<xsl:text>&#xa;</xsl:text>
		<xsl:text>&#9;</xsl:text>
		<xsl:text>edge [ color = red, fontcolor = red ];</xsl:text>
		<xsl:text>&#xa;</xsl:text>
		<xsl:text>&#9;</xsl:text>
		<xsl:text>node [ color = red, fontcolor = red ];</xsl:text>
		<xsl:text>&#xa;</xsl:text>
		<xsl:for-each select="world">
			<xsl:text>&#9;</xsl:text>
			<xsl:value-of select="concat('&quot;', @id, '&quot;')"/>
			<xsl:text> [ color = darkviolet, fontcolor = darkviolet, shape = doubleoctagon ];</xsl:text>
			<xsl:text>&#xa;</xsl:text>
		</xsl:for-each>
		<xsl:for-each select="frame">
			<xsl:text>&#9;</xsl:text>
			<xsl:value-of select="concat('&quot;', @id, '&quot;')"/>
			<xsl:text> [ color = black, fontcolor = black, shape = box ];</xsl:text>
			<xsl:text>&#xa;</xsl:text>
		</xsl:for-each>
		<xsl:for-each select="body">
			<xsl:text>&#9;</xsl:text>
			<xsl:value-of select="concat('&quot;', @id, '&quot;')"/>
			<xsl:text> [ color = blue, fillcolor = red, fontcolor = blue, shape = octagon ];</xsl:text>
			<xsl:text>&#xa;</xsl:text>
		</xsl:for-each>
		<xsl:for-each select="fixed">
			<xsl:text>&#9;</xsl:text>
			<xsl:value-of select="concat('&quot;', frame/a/@idref, '&quot;')"/>
			<xsl:text disable-output-escaping="yes"> -> </xsl:text>
			<xsl:value-of select="concat('&quot;', frame/b/@idref, '&quot;')"/>
			<xsl:text> [ </xsl:text>
			<xsl:value-of select="concat('label = &quot;', @id, '&quot;')"/>
			<xsl:text>, color = black, fontcolor = black ];</xsl:text>
			<xsl:text>&#xa;</xsl:text>
		</xsl:for-each>
		<xsl:for-each select="cylindrical|helical|prismatic|revolute|spherical">
			<xsl:text>&#9;</xsl:text>
			<xsl:value-of select="concat('&quot;', frame/a/@idref, '&quot;')"/>
			<xsl:text disable-output-escaping="yes"> -> </xsl:text>
			<xsl:value-of select="concat('&quot;', frame/b/@idref, '&quot;')"/>
			<xsl:text> [ </xsl:text>
			<xsl:value-of select="concat('label = &quot;', @id, '&quot;')"/>
			<xsl:text>, color = forestgreen, fontcolor = forestgreen ];</xsl:text>
			<xsl:text>&#xa;</xsl:text>
		</xsl:for-each>
		<xsl:text>}</xsl:text>
	</xsl:template>
</xsl:stylesheet>
