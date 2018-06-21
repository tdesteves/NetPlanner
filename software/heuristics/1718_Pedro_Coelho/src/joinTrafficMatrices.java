
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

import com.net2plan.interfaces.networkDesign.IAlgorithm;
import com.net2plan.interfaces.networkDesign.Net2PlanException;
import com.net2plan.interfaces.networkDesign.NetPlan;
import com.net2plan.utils.Triple;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.w3c.dom.*;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Result;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

/*
 * Algorithm that joins the demands of 2 traffic matrices into a single file for
 * loading on Net2Plan.
 *
 * @author Adolfo Oliveira
 * @version 1.0, June 2015
 */

public class joinTrafficMatrices implements IAlgorithm {

	private double sum[] = new double[5];

	public void sendToFile(String entryfile) {

		try {
			String file = "C:/Users/Asus/Documents/algorithms/src/";
			file += entryfile;
			File traffic = new File(file);
			FileOutputStream is = new FileOutputStream(traffic);
			OutputStreamWriter osw = new OutputStreamWriter(is);
			BufferedWriter w = new BufferedWriter(osw);
			String s = "";

			for (int i = 0; i < sum.length; i++) {
				s = String.valueOf(sum[i]);
				w.write(s);
				w.newLine();
			}
			w.close();

		} catch (IOException e) {
			System.err.println("Problem writing to the file traffic.txt");
		}

	}

	@Override
	public String executeAlgorithm(NetPlan netPlan, Map<String, String> algorithmParameters, Map<String, String> net2planParameters) {

		List<Integer> parameters = new ArrayList<Integer>();

		final int maxParameters = 5;
		final double traffic[] = { 1.25, 2.5, 10, 40, 100 };

		for (int i = 0; i < maxParameters; i++) {
			String path = algorithmParameters.get("trafficMatrix" + (i + 1));
			File f = new File(path);
			if (f.exists())
				parameters.add(i);
		}

		// check parameters.size()> 0
		if (parameters.size() == 0) {
			throw new Net2PlanException("Must have matrices to add");
		}

		double aux = 0;

		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		try {
			Document doc1 = factory.newDocumentBuilder().parse(new File(algorithmParameters.get("trafficMatrix" + (parameters.get(0) + 1))));
			NodeList list = doc1.getElementsByTagName("layer");
			Element element = (Element) list.item(0);
			int max_id = -1;

			NodeList entrylist = doc1.getElementsByTagName("demand");
			for (int i = 0; i < entrylist.getLength(); i++) {
				Node n = entrylist.item(i);
				String id_string = n.getAttributes().getNamedItem("id").getNodeValue();
				int id_int = Integer.parseInt(id_string);
				max_id = Math.max(max_id, id_int);
				String offeredTraffic_string = n.getAttributes().getNamedItem("offeredTraffic").getNodeValue();
				double offeredTraffic_int = Double.parseDouble(offeredTraffic_string) * traffic[parameters.get(0)];
				((Element) n).setAttribute("offeredTraffic", ((Double) offeredTraffic_int).toString());
				Element attribute = doc1.createElement("attribute");
				attribute.setAttribute("key", "ODU");
				attribute.setAttribute("value", "" + parameters.get(0));
				n.appendChild(attribute);

				sum[0] += Double.parseDouble(offeredTraffic_string);
				aux = Double.parseDouble(entrylist.item(entrylist.getLength() - 1).getAttributes()
						.getNamedItem("offeredTraffic").getNodeValue());
				aux = aux * traffic[parameters.get(0)];
			}

			max_id++;
			double aux2 = 0;

			for (int k = 1; k < parameters.size(); k++) {
				Document doc = factory.newDocumentBuilder().parse(new File(algorithmParameters.get("trafficMatrix" + (parameters.get(k) + 1))));
				entrylist = doc.getElementsByTagName("demand");
				// Add demands from ODU matrix
				for (int i = 0; i < entrylist.getLength(); i++) {

					Node n = doc1.importNode(entrylist.item(i), false);

					((Element) n).setAttribute("id", ((Integer) max_id).toString());
					String offeredTraffic_string = n.getAttributes().getNamedItem("offeredTraffic").getNodeValue();
					double offeredTraffic_int = Double.parseDouble(offeredTraffic_string) * traffic[parameters.get(k)];
					((Element) n).setAttribute("offeredTraffic", ((Double) offeredTraffic_int).toString());
					Element attribute = doc1.createElement("attribute");
					attribute.setAttribute("key", "ODU");
					attribute.setAttribute("value", "" + parameters.get(k));
					max_id++;
					element.appendChild(n);
					n.appendChild(attribute);

					aux2 += Double.parseDouble(offeredTraffic_string);
				}
				if (k < 6) {
					sum[k] = aux2;
					aux2 = 0;
				}

			}

			for (int i = 0; i < sum.length; i++) {
				System.out.println(sum[i] + " ");
			}

			sendToFile("traffic.txt");

			// Create matrix with demands
			Transformer t = TransformerFactory.newInstance().newTransformer();
			Result ouput = new StreamResult(new File(algorithmParameters.get("trafficMatrix6")));
			t.setOutputProperty(OutputKeys.INDENT, "yes");

			t.transform(new DOMSource(doc1), ouput);

		} catch (Throwable t) {
			t.printStackTrace();
		}

		return "Ok!";
	}

	@Override
	public List<Triple<String, String, String>> getParameters() {
		List<Triple<String, String, String>> parameters = new ArrayList<Triple<String, String, String>>();
		parameters.add(Triple.of("trafficMatrix1",
				"C:/Users/Asus/Documents/Universidade/Tese/heuristicas/without_protection/low_traffic/matrizes/ODU0.n2p",
				"Path to ODU0 traffic matrix"));
		parameters.add(Triple.of("trafficMatrix2",
				"C:/Users/Asus/Documents/Universidade/Tese/heuristicas/without_protection/low_traffic/matrizes/ODU1.n2p",
				"Path to ODU1 traffic matrix"));
		parameters.add(Triple.of("trafficMatrix3",
				"C:/Users/Asus/Documents/Universidade/Tese/heuristicas/without_protection/low_traffic/matrizes/ODU2.n2p",
				"Path to ODU2 traffic matrix"));
		parameters.add(Triple.of("trafficMatrix4",
				"C:/Users/Asus/Documents/Universidade/Tese/heuristicas/without_protection/low_traffic/matrizes/ODU3.n2p",
				"Path to ODU3 traffic matrix"));
		parameters.add(Triple.of("trafficMatrix5",
				"C:/Users/Asus/Documents/Universidade/Tese/heuristicas/without_protection/low_traffic/matrizes/ODU4.n2p",
				"Path to ODU4 traffic matrix"));
		parameters.add(Triple.of("trafficMatrix6",
				"C:/Users/Asus/Documents/Universidade/Tese/heuristicas/without_protection/low_traffic/matrizes/ODUs.n2p",
				"Path to file with new demands."));
		return parameters;
	}

	@Override
	public String getDescription() {
		return "Joins the demands of the 5 ODU traffic matrices into a single file for loading on Net2Plan.";
	}

}