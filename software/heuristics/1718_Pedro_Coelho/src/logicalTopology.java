
/* ******************************************************************************
 * Copyright (c) 2013-2014 Pablo Pavon-Marino, Jose-Luis Izquierdo-Zaragoza.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v3
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl.html
 *
 * Contributors:
 *     Pablo Pavon-Marino, Jose-Luis Izquierdo-Zaragoza - initial API and implementation
 ***************************************************************************** */

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.net2plan.interfaces.networkDesign.Demand;
import com.net2plan.interfaces.networkDesign.IAlgorithm;
import com.net2plan.interfaces.networkDesign.Link;
import com.net2plan.interfaces.networkDesign.Net2PlanException;
import com.net2plan.interfaces.networkDesign.NetPlan;
import com.net2plan.utils.StringUtils;
import com.net2plan.utils.Triple;
//import cern.colt.matrix.tdouble.DoubleFactory1D;
//import cern.colt.matrix.tdouble.DoubleFactory2D;
//import cern.colt.matrix.tdouble.DoubleMatrix1D;
//import cern.colt.matrix.tdouble.DoubleMatrix2D;
//import cern.jet.math.tdouble.DoubleFunctions;
import com.net2plan.interfaces.networkDesign.NetworkLayer;
import com.net2plan.interfaces.networkDesign.Node;
import com.net2plan.interfaces.networkDesign.Route;
import com.net2plan.utils.Constants.RoutingType;
import com.net2plan.utils.InputParameter;

/**
 * @author Vasco Braz, Adolfo Oliveira
 * @version 2.0, May 2016
 */

public class logicalTopology implements IAlgorithm {
	
	private NetworkLayer lowerLayer, upperLayer;
	private int maxOpticalReach;

	public void sendToFile(String entryfile) {

		try {
			String file = "C:/Users/Asus/Documents/algorithms/src/";
			file += entryfile;
			File opticalReach = new File(file);
			FileOutputStream is = new FileOutputStream(opticalReach);
			OutputStreamWriter osw = new OutputStreamWriter(is);
			BufferedWriter w = new BufferedWriter(osw);
			String s = "";

			s = String.valueOf(maxOpticalReach);
			w.write(s);
			w.close();

		} catch (IOException e) {
			System.err.println("Problem writing to the file opticalReach.txt");
		}
	}

	@Override
	public String executeAlgorithm(NetPlan netPlan, Map<String, String> algorithmParameters, Map<String, String> net2planParameters) {

		/*
		 * Initialize all InputParameter objects defined in this object (this uses Java
		 * reflection)
		 */
		InputParameter.initializeAllInputParameterFieldsOfObject(this, algorithmParameters);

		String logicalTopology = algorithmParameters.get("logicalTopology");

		final int N = netPlan.getNumberOfNodes();
		if (N == 0)
			throw new Net2PlanException("This algorithm requires a topology with nodes");

		if (netPlan.isMultilayer()) {
			NetworkLayer l1 = netPlan.getNetworkLayer(1);
			netPlan.removeNetworkLayer(l1);
		}

		if (netPlan.isSingleLayer() && logicalTopology.equalsIgnoreCase("Opaque")) {

			lowerLayer = netPlan.getNetworkLayerDefault();
			upperLayer = netPlan.addLayerFrom(lowerLayer);
			netPlan.setRoutingType(RoutingType.HOP_BY_HOP_ROUTING, upperLayer);
			lowerLayer.setName("Physical Topology");
			upperLayer.setName("Logical Topology Opaque");
			upperLayer.setDescription("Opaque Logical Topology");

			// Save the demands in the upper layer, and remove them from the lower layer*/
			// for (Demand d : netPlan.getDemands (lowerLayer)) netPlan.addDemand(d.getIngressNode(), d.getEgressNode(), d.getOfferedTraffic() , null , upperLayer);
		}

		if (netPlan.isSingleLayer() && logicalTopology.equalsIgnoreCase("Transparent")) {

			this.lowerLayer = netPlan.getNetworkLayerDefault();
			lowerLayer.setName("Physical Topology");
			this.upperLayer = netPlan.addLayer("Logical Topology Transparent", "Upper layer of the design", "ODU", "ODU", null);
			upperLayer.setDescription("Transparent Logical Topology");
			netPlan.removeAllLinks(upperLayer);

			for (Node i : netPlan.getNodes()) {
				for (Node j : netPlan.getNodes()) {
					if (i.getIndex() != j.getIndex()) {
						netPlan.addLink(i, j, 0, netPlan.getNodePairEuclideanDistance(i, j), 200000, null, upperLayer);
					}
				}
			}

			// Save the demands in the upper layer, and remove them from the lower layer*/
			// for (Demand d : netPlan.getDemands (lowerLayer)) netPlan.addDemand(d.getIngressNode(), d.getEgressNode(), d.getOfferedTraffic() , null , upperLayer);

		}

		if (netPlan.isSingleLayer() && logicalTopology.equalsIgnoreCase("Translucent")) {

			int maximumOpticalReach = Integer.parseInt(algorithmParameters.get("maximumOpticalReach"));
			maxOpticalReach = maximumOpticalReach;

			sendToFile("opticalReach.txt");

			this.lowerLayer = netPlan.getNetworkLayerDefault();
			lowerLayer.setName("Physical Topology");
			this.upperLayer = netPlan.addLayer("Logical Topology Translucent", "Upper layer of the design", "ODU", "ODU", null);
			upperLayer.setDescription("Translucent Logical Topology" + " - Maximum Optical Reach = " + maximumOpticalReach + " km");
			netPlan.removeAllLinks(upperLayer);

			for (Node i : netPlan.getNodes()) {
				for (Node j : netPlan.getNodes()) {
					if (i.getIndex() != j.getIndex()) {
						if (netPlan.getNodePairEuclideanDistance(i, j) <= maximumOpticalReach) {
							netPlan.addLink(i, j, 0, netPlan.getNodePairEuclideanDistance(i, j), 200000, null, upperLayer);
						}
					}
				}
			}
			
			// Save the demands in the upper layer, and remove them from the lower layer*/
			// for (Demand d : netPlan.getDemands (lowerLayer)) netPlan.addDemand(d.getIngressNode(), d.getEgressNode(), d.getOfferedTraffic(), null, upperLayer);

		}
		
		return "Ok!";
	}

	@Override
	public String getDescription() {
		
		StringBuilder description = new StringBuilder();
		String NEW_LINE = StringUtils.getLineSeparator();
		
		description.append("Logical Topology:");
		description.append(NEW_LINE);
		description.append("Opaque");
		description.append(NEW_LINE);
		description.append("Transparent");
		description.append(NEW_LINE);
		description.append("Translucent");
		description.append(NEW_LINE);
		description.append("");
		description.append(NEW_LINE);
		description.append("This algorithm creates the logical topology on another layer based on the type of transport mode chosen.");
		
		return description.toString();
	}

	@Override
	public List<Triple<String, String, String>> getParameters() {
		
		List<Triple<String, String, String>> parameters = new ArrayList<Triple<String, String, String>>();
		parameters.add(Triple.of("logicalTopology", "Translucent", "Logical topology type."));
		parameters.add(Triple.of("maximumOpticalReach", "1000", "Maximum optical reach in km."));
	
		return parameters;
	}
}